# FOC电机控制核心技术文档

本文档整理了FOC电机控制系统的三大核心技术模块：ADC采样、角度来源、控制模式与闭环结构。

---

## 目录

1. [ADC采样系统](#一adc采样系统)
2. [角度来源](#二角度来源)
3. [控制模式与闭环结构](#三控制模式与闭环结构)

---

## 一、ADC采样系统

### 1.1 硬件配置

#### ADC硬件连接

| ADC通道 | GPIO引脚 | 采样内容 |
|---------|---------|---------|
| ADC1_IN1 | PA0 | U相电流 |
| ADC1_IN2 | PA1 | V相电流 |
| ADC1_IN3 | PA2 | W相电流 |
| ADC1_IN4 | PA3 | 母线电压 |

#### 关键配置参数

```c
// 分辨率：12位 → 量化范围 0~4095
hadc1.Init.Resolution = ADC_RESOLUTION_12B;

// 数据对齐：右对齐
hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;

// 扫描模式：使能（多通道采样）
hadc1.Init.ScanConvMode = ADC_SCAN_ENABLE;

// 连续转换模式：使能
hadc1.Init.ContinuousConvMode = ENABLE;

// 采样时间：12.5个ADC时钟周期
sConfigInjected.InjectedSamplingTime = ADC_SAMPLETIME_12CYCLES_5;

// 触发源：TIM1_TRGO（PWM更新事件）
sConfigInjected.ExternalTrigInjecConv = ADC_EXTERNALTRIGINJEC_T1_TRGO;
```

#### 采样触发机制

```
PWM周期开始 → TIM1_TRGO触发 → ADC注入通道采样 → 4通道依次转换 → ADC中断
```

**关键点**：ADC采样与PWM同步，在PWM周期固定位置采样，避免开关噪声干扰。

### 1.2 电流采样与换算

#### 电流采样电路原理

```
电机电流 → 采样电阻 → 运放放大 → ADC采样
```

**基础知识**：
- **采样电阻**：将电流转换为电压（V = I × R）
- **运放放大**：将小信号放大到ADC可测范围
- **偏置电压**：将双极性电流信号转换为单极性电压信号

#### 电流换算公式

```c
// 原始ADC值 → 实际电流值
adcvalue->Iu = ((float)adcvalue->Iu_Sample * 3.3f / 4096 - adcvalue->adc_refer) * adcvalue->gain_cur;
adcvalue->Iv = ((float)adcvalue->Iv_Sample * 3.3f / 4096 - adcvalue->adc_refer) * adcvalue->gain_cur;
adcvalue->Iw = ((float)adcvalue->Iw_Sample * 3.3f / 4096 - adcvalue->adc_refer) * adcvalue->gain_cur;
```

**公式拆解**：

| 步骤 | 计算 | 说明 |
|------|------|------|
| 1. ADC转电压 | `ADC × 3.3 / 4096` | 12位ADC，参考电压3.3V |
| 2. 去除偏置 | `- adc_refer` | 减去零点偏置电压（1.65V） |
| 3. 电流换算 | `× gain_cur` | 根据硬件电路系数转换为实际电流 |

**参数配置**：

```c
#define ADCREFER  1.65          // ADC零点偏置电压（3.3V/2）
#define ADCCURGAIN  100.0f/16.5f  // 电流增益系数
#define ADCUDCGAIN  25.0f       // 母线电压增益系数
```

### 1.3 零漂校准

#### 零漂产生原因

| 原因 | 说明 |
|------|------|
| **运放偏移** | 运放输入失调电压 |
| **ADC偏移** | ADC内部零点偏移 |
| **温度漂移** | 温度变化导致的偏移变化 |
| **三相不平衡** | 三相电路参数不一致 |

#### 零漂校准算法

```c
// 上电后进行零漂校准（5000次采样，约0.5秒）
if (adcvalue->ADCcount < 5000)
{
    adcvalue->adcstateflag = 1;  // 校准中
    adcvalue->ADCcount++;

    // 滑动平均滤波计算零点偏置
    adcvalue->Iu_Sample_offer = adcvalue->Iu_Sample_offer * 0.998f + (float)adcvalue->Iu * 0.002f;
    adcvalue->Iv_Sample_offer = adcvalue->Iv_Sample_offer * 0.998f + (float)adcvalue->Iv * 0.002f;
    adcvalue->Iw_Sample_offer = adcvalue->Iw_Sample_offer * 0.998f + (float)adcvalue->Iw * 0.002f;
}
else
{
    adcvalue->adcstateflag = 2;  // 校准完成
}
```

**算法原理**：
```
y(n) = y(n-1) × 0.998 + x(n) × 0.002
```

这是一阶低通滤波器，用于平滑零点偏置值。

#### 零漂补偿

```c
if (adcvalue->adcstateflag == 2)  // 校准完成后
{
    // 实际电流 = 原始电流 - 零点偏置
    current->Ia = (adcvalue->Iu - adcvalue->Iu_Sample_offer);
    current->Ib = (adcvalue->Iv - adcvalue->Iv_Sample_offer);
    current->Ic = (adcvalue->Iw - adcvalue->Iw_Sample_offer);
}
```

#### 三相电流不平衡校准

```c
// 计算三相平均电流
float average = (Iasum + Ibsum + Icsum) / 3;

// 计算各相补偿系数
adcvalue->currentoffera = (average / Iasum);
adcvalue->currentofferb = (average / Ibsum);
adcvalue->currentofferc = (average / Icsum);

// 应用补偿
current->Ia = current->Ia * adcvalue->currentoffera;
current->Ib = current->Ib * adcvalue->currentofferb;
current->Ic = current->Ic * adcvalue->currentofferc;
```

### 1.4 涉及的基础知识

#### 模数转换（ADC）基础

| 概念 | 说明 |
|------|------|
| **分辨率** | 12位ADC可表示 2^12 = 4096 个量化等级 |
| **量化误差** | ±0.5 LSB = ±0.5/4096 × 3.3V ≈ ±0.4mV |
| **采样率** | 由PWM频率决定，通常10kHz~20kHz |
| **采样时间** | 12.5个ADC时钟周期，确保采样稳定 |

#### PWM同步采样

```
PWM周期
  │
  ├─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┬─┤
  │   │   │   │   │   │   │   │   │   │   │   │   │   │   │   │
  ↑       ↑       ↑       ↑       ↑       ↑       ↑
  │       │       │       │       │       │       │
 采样点1  采样点2  采样点3  采样点4  采样点5  采样点6  采样点7
```

**为什么要同步采样？**
- 避免开关噪声干扰
- 在PWM导通中心采样，电流稳定
- 提高采样精度

#### 三相电流关系

根据基尔霍夫电流定律：
```
Ia + Ib + Ic = 0
```

因此：只需要采样两相电流，第三相可以计算得到。

### 1.5 完整采样流程图

```
┌─────────────────────────────────────────────────────────────┐
│                    PWM周期开始                              │
└──────────────────────┬──────────────────────────────────────┘
                       │
                       ▼
              ┌────────────────┐
              │  TIM1_TRGO触发 │
              └────────┬───────┘
                       │
                       ▼
              ┌────────────────┐
              │ ADC注入通道采样 │
              │  CH1: U相电流  │
              │  CH2: V相电流  │
              │  CH3: W相电流  │
              │  CH4: 母线电压  │
              └────────┬───────┘
                       │
                       ▼
              ┌────────────────┐
              │ ADC转换完成中断 │
              └────────┬───────┘
                       │
                       ▼
        ┌──────────────┼──────────────┐
        │              │              │
        ▼              ▼              ▼
┌──────────────┐ ┌──────────────┐ ┌──────────────┐
│ 读取ADC原始值 │ │ 编码器读取   │ │ 执行FOC控制  │
│ aADCxINJ...  │ │ encoderCount │ │ motor_control│
└──────┬───────┘ └──────────────┘ └──────────────┘
       │
       ▼
┌─────────────────────────────────────────────┐
│          电流换算与零漂补偿                  │
│  1. ADC → 电压：ADC × 3.3 / 4096           │
│  2. 去偏置：电压 - 1.65V                    │
│  3. 电流换算：× gain_cur                    │
│  4. 零漂补偿：原始值 - 零点偏置              │
│  5. 不平衡补偿：× currentoffer系数           │
└─────────────────────────────────────────────┘
```

---

## 二、角度来源

### 2.1 三种角度来源概述

| 角度来源 | 模式 | 适用场景 | 精度 |
|---------|------|---------|------|
| **开环角度自增** | Theta_mode = 0 | 电机启动、低速测试 | 低 |
| **编码器角度** | Theta_mode = 1 | 高速高精度控制 | 高 |
| **SMO无感角度** | Theta_mode = 5 | 无传感器控制 | 中高 |

### 2.2 开环角度自增（Theta_mode = 0）

#### 实现原理

```c
if (mode->Theta_mode == 0)
{
    // 角度递增速度限制
    if (theta->TetaInc > 100.0f)
    {
        theta->TetaInc = 100.0f;
    }

    // 角度累加：TetaInc × 0.036 = 每周期增加的角度
    theta->Tetatmp += (theta->TetaInc * 0.036f);

    // 角度范围限制 0~360°
    if (theta->Tetatmp > 360)
        theta->Tetatmp -= 360;
    else if (theta->Tetatmp < 0)
        theta->Tetatmp += 360;

    theta->TetaRef = (uint16_t)theta->Tetatmp;
}
```

#### 核心公式

```
TetaRef(n) = TetaRef(n-1) + TetaInc × 0.036
```

**参数说明**：
- `TetaInc`：角速度增量（可配置）
- `0.036`：换算系数（10°/1000Hz = 0.01°/ms）

#### 适用场景

- **电机启动**：无传感器模式下从零速启动
- **IF启动法**：注入恒定电流使电机定位，然后逐步加速
- **测试验证**：验证FOC算法基本功能

### 2.3 编码器角度（Theta_mode = 1）

#### 硬件配置

- 使用TIM3作为编码器计数器
- 支持正交解码模式（4倍频）

#### 实现原理

```c
void coderpro(THETA_s *theta, uint16_t coderCount)
{
    // 方向处理：根据motordir决定正反转
    if (theta->motordir == 1)
        theta->encoder = theta->OnePulseNum - coderCount;
    else
        theta->encoder = coderCount;

    theta->OldTheta = theta->NewTheta;
    theta->NewTheta = (int16_t)theta->encoder;

    // 机械角度转换
    theta->realencoder = theta->encoder % theta->OnePulseNum;
    theta->MotorTeta = (int16_t)((int32_t)(theta->realencoder * 360) / theta->OnePulseNum);

    // 电气角度转换（考虑极对数）
    theta->encoderelc = theta->encoder % (theta->OnePulseNum / theta->MotorPn);
    theta->Teta = (int16_t)((theta->encoderelc * 360) / (theta->OnePulseNum / theta->MotorPn));
}
```

#### 核心公式

| 计算项 | 公式 |
|--------|------|
| 机械角度 | `MotorTeta = (encoder × 360) / OnePulseNum` |
| 电气角度 | `Teta = MotorTeta × Pn` |
| 转速计算 | `Speed = Δencoder × 60 × 1000 / OnePulseNum` |

#### 关键参数

```c
#define Pn 7                    // 电机极对数
#define PluseNum 16384          // 编码器每转脉冲数（4倍频后）
#define ENCODER 2               // 编码器选择：0无 1有 2无感
```

#### 基础知识

| 概念 | 说明 |
|------|------|
| **增量式编码器** | AB相正交输出，4倍频后精度提升4倍 |
| **机械角度** | 电机实际旋转角度（0~360°） |
| **电气角度** | 电角度 = 机械角度 × 极对数 |
| **正交解码** | 通过AB相信号相位判断方向和计数 |

### 2.4 无感SMO角度（Theta_mode = 5）

#### 2.4.1 滑模观测器原理

SMO（Sliding Mode Observer，滑模观测器）是一种基于滑模变结构控制理论的状态观测器，通过观测电机的反电动势来估算转子位置和速度。

##### 2.4.1.1 电机数学模型

在**α-β静止坐标系**下，永磁同步电机的电压方程为：

```
┌─────────────────────────────────────────────────────────┐
│ 电压方程（α-β坐标系）                                    │
│                                                         │
│ Vα = Rs×Iα + Ls×dIα/dt + Eα                             │
│ Vβ = Rs×Iβ + Ls×dIβ/dt + Eβ                             │
│                                                         │
│ 其中：                                                  │
│   Eα = -ω×Ψf×sin(θ)    ← α轴反电动势                    │
│   Eβ =  ω×Ψf×cos(θ)    ← β轴反电动势                    │
│                                                         │
│ 参数说明：                                               │
│   Rs    : 定子电阻 (Ω)                                   │
│   Ls    : 定子电感 (H)                                   │
│   ω     : 电角速度 (rad/s)                              │
│   Ψf    : 永磁体磁链 (Wb)                               │
│   θ     : 转子电角度 (rad)                               │
└─────────────────────────────────────────────────────────┘
```

**关键特性**：反电动势与电机转速成正比（E = k×ω），这是SMO能够工作的物理基础。

##### 2.4.1.2 滑模观测器结构

```
┌─────────────────────────────────────────────────────────────┐
│                    滑模观测器结构                            │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│   真实系统              观测器                              │
│   ┌─────────┐          ┌─────────┐                         │
│   │ 电机    │          │ 状态观测 │                         │
│   │ Vα,Vβ → │          │ Vα,Vβ → │                         │
│   │  Iα,Iβ  │          │ EstIα,  │                         │
│   │  (真实) │          │ EstIβ   │                         │
│   └────┬────┘          └────┬────┘                         │
│        │                    │                              │
│        └──────────┬─────────┘                              │
│                   ↓                                        │
│          ┌─────────────────┐                              │
│          │ 电流误差计算     │                              │
│          │ Iα_err = EstIα - Iα                            │
│          │ Iβ_err = EstIβ - Iβ                            │
│          └────────┬────────┘                              │
│                   ↓                                        │
│          ┌─────────────────┐                              │
│          │ 滑模控制律      │                              │
│          │ Zα = K×sign(Iα_err)                            │
│          │ Zβ = K×sign(Iβ_err)                            │
│          └────────┬────────┘                              │
│                   ↓                                        │
│          ┌─────────────────┐                              │
│          │ 反电动势估算    │                              │
│          │ Eα ≈ Zα (低通滤波后)                           │
│          │ Eβ ≈ Zβ (低通滤波后)                           │
│          └────────┬────────┘                              │
│                   ↓                                        │
│          ┌─────────────────┐                              │
│          │ 角度计算        │                              │
│          │ θ = arctan2(Eβ, Eα)                            │
│          └─────────────────┘                              │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

##### 2.4.1.3 滑模控制律

滑模观测器的核心是**不连续控制律**：

```
Zα = Kslide × sign(Iα_error)
Zβ = Kslide × sign(Iβ_error)
```

其中 `sign()` 是符号函数：
```c
sign(x) =  1  if x > 0
          -1  if x < 0
           0  if x = 0
```

**工作原理**：当观测电流与真实电流存在误差时，滑模控制律会产生一个不连续的控制信号，迫使观测器状态快速收敛到真实状态。

##### 2.4.1.4 稳定性分析

滑模观测器的稳定性基于**李雅普诺夫稳定性理论**：

1. **滑模面定义**：s = EstI - I = 0
2. **到达条件**：s × ds/dt < 0
3. **等效控制**：当系统到达滑模面后，Zα, Zβ 近似等于反电动势

#### 2.4.2 实现步骤

**步骤1：电流观测**

```c
// 观测器状态方程
pV->EstIalpha = pV->EstIalpha * pV->Fsmopos + (pV->Valpha - pV->Ealpha - pV->Zalpha) * pV->Gsmopos;
pV->EstIbeta = pV->EstIbeta * pV->Fsmopos + (pV->Vbeta - pV->Ebeta - pV->Zbeta) * pV->Gsmopos;
```

**步骤2：电流误差计算**

```c
pV->IalphaError = pV->EstIalpha - pV->Ialpha;
pV->IbetaError = pV->EstIbeta - pV->Ibeta;
```

**步骤3：滑模控制**

```c
// 饱和限幅
IalphaError_Limit = Limit_Sat(pV->IalphaError, pV->E0, -pV->E0);
IbetaError_Limit = Limit_Sat(pV->IbetaError, pV->E0, -pV->E0);

// 滑模面计算
pV->Zalpha = IalphaError_Limit * pV->Kslide;
pV->Zbeta = IbetaError_Limit * pV->Kslide;
```

**步骤4：反电动势估算**

```c
// 低通滤波提取反电动势
pV->Ealpha = pV->Ealpha + (pV->Zalpha - pV->Ealpha) * pV->Kslf;
pV->Ebeta = pV->Ebeta + (pV->Zbeta - pV->Ebeta) * pV->Kslf;
```

**步骤5：反正切计算角度**

使用查表法实现反正切：

```c
void IQAtan_Cale(p_IQAtan pV)
{
    // 基于查表的快速反正切算法
    pV->IQTan = _IQdiv(Abs(pV->Alpha), Abs(pV->Beta));

    // 二分查找查表
    if (IQAtan2_Table[i + 128] <= pV->IQTan) i += 128;
    if (IQAtan2_Table[i + 64] <= pV->IQTan) i += 64;
    // ... 继续二分查找

    // 根据象限确定最终角度
    if (pV->Alpha > 0 && pV->Beta > 0)
        pV->IQAngle = i;
    // ... 其他象限处理
}
```

#### SMO参数初始化

```c
void SMO_Pare_init(void)
{
    SMO_MotorPare.Rs = 0.12f;        // 定子电阻(Ω)
    SMO_MotorPare.Ls = 0.0003f;      // 定子电感(H)
    SMO_MotorPare.Ib = 10.0f;        // 电流基准
    SMO_MotorPare.Vb = VOLTAGE_LIM;  // 电压基准
    SMO_MotorPare.Ts = 0.0001f;      // 采样时间(10kHz)
    SMO_MotorPare.POLES = Pn;        // 极对数

    // 观测器系数计算
    SMO_MotorPare.Fsmopos = exp((-Rs/Ls) * Ts);
    SMO_MotorPare.Gsmopos = (Vb/Ib) * (1/Rs) * (1 - Fsmopos);

    // 滑模参数
    Angle_SMOPare.Kslide = 0.17f;    // 滑模增益
    Angle_SMOPare.Kslf = 0.06f;      // 低通滤波系数
}
```

#### 无感启动流程

```c
if (motorflag_s.Conrorl_mode == 5) // 无感模式
{
    if (theta_s.flag == 1 || theta_s.flag == 2) // IF启动/混合模式
    {
        // 注入恒定电流启动
        current_s.Idr = paraset_s.openloop_Idref;
        current_s.Iqr = paraset_s.openloop_Iqref;
    }
    else if (theta_s.flag == 3) // 速度闭环模式
    {
        // 使用SMO估算角度
        current_s.Idr = 0;
        current_s.Iqr = speed_s.SPEED_OUT;
    }
}
```

**启动阶段划分**：

| 阶段 | flag值 | 速度范围 | 角度来源 |
|------|--------|---------|---------|
| IF启动 | 1 | 0~250 RPM | 开环角度自增 |
| 混合 | 2 | 250~300 RPM | SMO角度（带补偿） |
| 速度闭环 | 3 | >300 RPM | SMO角度 |

#### 速度估算

```c
void SMO_Speedcale(void)
{
    // 角度差分计算速度
    Speed_estPare.Speed_ele_angle = Speed_estPare.ele_angle - Speed_estPare.old_ele_angleIQ;

    // 一阶低通滤波
    Speed_estPare.Speed_ele_angleIQFitter = SpeedK2 * Speed_ele_angleIQFitter + SpeedK1 * Speed_ele_angle;

    // 转换为RPM
    Speed_estPare.Speed_RPM = Speed_ele_angleIQFitter * 1000 * 60 / 1024 / POLES;
}
```

#### 2.4.6 涉及的基础知识

##### 2.4.6.1 滑模变结构控制理论

**核心概念**：
- **滑模面**：系统状态空间中的一个超平面，系统状态被强制到达并保持在该平面上
- **可达性**：系统状态从任意初始位置在有限时间内到达滑模面
- **不变性**：一旦到达滑模面，系统状态将保持在该平面上
- **鲁棒性**：对参数变化和外部干扰具有很强的不敏感性

**数学描述**：
```
s(x) = 0  ← 滑模面
ds/dt < 0 ← 到达条件
```

##### 2.4.6.2 反电动势原理

**产生机制**：当电机转子旋转时，永磁体的磁场切割定子绕组，根据法拉第电磁感应定律产生感应电动势。

**特性**：
| 特性 | 说明 |
|------|------|
| **幅值** | E = k × ω（与转速成正比） |
| **频率** | 与电机转速和极对数相关 |
| **波形** | 理想情况下为正弦波 |

**反电动势常数**：
```
Ke = E / ω = Ψf × Pn
```
其中 `Pn` 是极对数，`Ψf` 是永磁体磁链。

##### 2.4.6.3 坐标系变换

**Clarke变换**（三相→两相静止）：
```
┌     ┐     ┌           ┐ ┌   ┐
│ Iα  │     │ 1   0      │ │ Ia │
│     │ =   │           │ │    │
│ Iβ  │     │ 1/√3  2/√3 │ │ Ib │
└     ┘     └           ┘ └    ┘
```

**Park变换**（两相静止→两相旋转）：
```
┌     ┐     ┌             ┐ ┌     ┐
│ Id  │     │ cosθ  sinθ  │ │ Iα  │
│     │ =   │             │ │     │
│ Iq  │     │ -sinθ cosθ  │ │ Iβ  │
└     ┘     └             ┘ └     ┘
```

##### 2.4.6.4 IF启动法

**原理**：注入恒定的直轴电流（Id）使电机转子锁定到指定位置，然后逐步增加交轴电流（Iq）使电机加速。

**启动流程**：
```
1. 注入Id电流 → 转子定位
2. 逐步增加Iq → 电机加速
3. 速度达到阈值 → 切换到SMO控制
```

**优点**：
- 零速启动能力
- 启动平稳
- 不需要位置传感器

**缺点**：
- 启动过程需要时间
- 需要准确的电机参数

##### 2.4.6.5 反正切查表算法

**原理**：使用查找表（LUT）实现快速反正切计算，避免浮点运算。

**算法步骤**：
1. 计算正切值：tanθ = |Eα| / |Eβ|
2. 使用二分查找在查找表中定位
3. 根据象限确定最终角度

**查找表结构**：
- 256项反正切值表
- 精度：约1.4°（360°/256）
- 存储格式：IQ格式（定点数）

##### 2.4.6.6 一阶低通滤波器

**传递函数**：
```
H(s) = Kslf / (s + Kslf)
```

**离散实现**：
```
y(n) = y(n-1) + Kslf × (x(n) - y(n-1))
```

**时间常数**：
```
τ = 1 / Kslf
```

##### 2.4.6.7 参数说明

| 参数 | 符号 | 取值范围 | 作用 |
|------|------|---------|------|
| **滑模增益** | Kslide | 0.1~0.5 | 影响观测器收敛速度和噪声 |
| **滤波系数** | Kslf | 0.01~0.1 | 反电动势低通滤波系数 |
| **限幅值** | E0 | 1~5V | 电流误差限幅 |
| **定子电阻** | Rs | 实测值 | 电机参数 |
| **定子电感** | Ls | 实测值 | 电机参数 |

**参数调优指南**：
- **Kslide增大**：响应更快，但噪声更大
- **Kslf增大**：反电动势提取更快，但噪声更大
- **E0增大**：允许更大的电流误差

### 2.5 三种角度来源对比

| 特性 | 开环角度 | 编码器角度 | SMO无感角度 |
|------|---------|-----------|------------|
| **硬件需求** | 无 | 增量式编码器 | 无 |
| **精度** | 低（依赖时间精度） | 高（取决于编码器分辨率） | 中高 |
| **适用速度** | 低速启动 | 全速度范围 | 中高速（>300RPM） |
| **成本** | 低 | 高 | 低 |
| **复杂度** | 简单 | 中等 | 高 |
| **启动方式** | IF启动 | 直接启动 | IF启动→SMO切换 |

---

## 三、控制模式与闭环结构

### 3.1 控制模式概述

项目支持**5种控制模式**：

| 模式 | 控制目标 | 闭环结构 | 适用场景 |
|------|---------|---------|---------|
| **模式1** | 电压开环 | 无闭环 | 测试、调试 |
| **模式2** | 电流闭环 | 电流环 | 力矩控制 |
| **模式3** | 速度闭环 | 速度环+电流环 | 速度控制 |
| **模式4** | 位置闭环 | 位置环+速度环+电流环 | 位置控制 |
| **模式5** | 无感速度闭环 | SMO+速度环+电流环 | 无传感器控制 |

### 3.2 模式1：电压开环（Conrorl_mode = 1）

**控制目标**：直接给定 dq 轴电压指令

**闭环结构**：无闭环

```c
if (motorflag_s.Conrorl_mode == 1)  // 电压开环
{
    voltage_s.Udr = paraset_s.Ud_ref;
    voltage_s.Uqr = paraset_s.Uq_ref;
}
```

**特点**：
- 无反馈控制
- 直接设置 `Ud_ref` 和 `Uq_ref`
- 用于验证SVPWM和基本硬件功能

### 3.3 模式2：电流闭环（Conrorl_mode = 2）

**控制目标**：跟踪 dq 轴电流指令

**闭环结构**：单电流环

```
电流指令(Idr, Iqr) → PI调节 → 电压输出(Ud, Uq) → SVPWM → 电机
       ↑
       │
       └── 电流反馈(Ia, Ib, Ic) → Park变换 → (Id, Iq)
```

**实现代码**：

```c
if (motorflag_s.Conrorl_mode == 2)  // 电流闭环
{
    current_s.Idr = paraset_s.Id_ref;
    current_s.Iqr = paraset_s.Iq_ref;
}
```

**电流环PI调节**：

```c
void runcur_process(CURRENT_s *current, VOLTAGE_s *voltage, MOTORFLAG_s *mode)
{
    // d轴电流PI调节
    current->IdErr = current->Idr - current->Id;
    current->IdErr = Limit_Sat(current->IdErr, 1000, -1000);

    voltage->Udr = (current->IdErr * current->IdKp) + current->Idki_out;

    // 积分项限幅
    current->Idki_out += current->IdErr * current->IdKi;
    current->Idki_out = Limit_Sat(current->Idki_out, 3000, -3000);

    // q轴电流PI调节（类似）
    current->IqErr = current->Iqr - current->Iq;
    voltage->Uqr = (current->IqErr * current->IqKp) + current->Iqki_out;
}
```

**参数配置**：

```c
#define IDKP  1.5f    // d轴电流环比例系数
#define IDKI  0.05f   // d轴电流环积分系数
#define IQKP  1.5f    // q轴电流环比例系数
#define IQKI  0.05f   // q轴电流环积分系数
```

### 3.4 模式3：速度闭环（Conrorl_mode = 3）

**控制目标**：跟踪速度指令

**闭环结构**：速度环（外环）+ 电流环（内环）

```
速度指令(SPEED_REF) → 速度PI → q轴电流指令(Iqr) → 电流PI → 电压(Ud, Uq)
       ↑                                                       │
       │                                                       ↓
       └────────────────── 速度反馈(SPEED_FB) ← 角度差分 ← 电机
```

**实现代码**：

```c
if (motorflag_s.Conrorl_mode == 3)  // 速度闭环
{
    current_s.Idr = 0;
    current_s.Iqr = speed_s.SPEED_OUT;
    speed_s.SPEED_KP = paraset_s.speed_kp;
    speed_s.SPEED_KI = paraset_s.speed_ki;
}
```

**速度环PI调节**：

```c
void Speed_PI(SPEED_s *speed)
{
    // 速度误差计算
    speed->SPEED_ERR = speed->SPEED_REF - speed->SPEED_FB;
    speed->SPEED_ERR = Limit_Sat(speed->SPEED_ERR, 1000, -1000);

    // 比例项
    speed->SPEED_OUT = speed->SPEED_ERR * speed->SPEED_KP;

    // 积分项（带积分分离）
    if (fabs(speed->SPEED_ERR) > 100)
        speed->SPEED_KI_OUT = 0;  // 大误差时关闭积分
    else
        speed->SPEED_KI_OUT += speed->SPEED_ERR * speed->SPEED_KI;

    // 积分限幅
    speed->SPEED_KI_OUT = Limit_Sat(speed->SPEED_KI_OUT, 500, -500);

    // 总输出
    speed->SPEED_OUT += speed->SPEED_KI_OUT;

    // 输出限幅（电流限制）
    speed->SPEED_OUT = Limit_Sat(speed->SPEED_OUT, CURRENT_LIM, -CURRENT_LIM);
}
```

**参数配置**：

```c
#define SPEED_KP  0.5f    // 速度环比例系数
#define SPEED_KI  0.02f   // 速度环积分系数
```

**速度计算**：

```c
// 编码器模式：通过角度差分计算速度
speed->SPEED_FB = theta->ThetaSum * (60.0f * 1000.0f / (float)theta->OnePulseNum);

// SMO模式：使用观测器估算速度
speed->SPEED_FB = Angle_SMOPare.Est_Speed;
```

### 3.5 模式4：位置闭环（Conrorl_mode = 4）

**控制目标**：跟踪位置角度指令

**闭环结构**：位置环（外环）+ 速度环（中环）+ 电流环（内环）

```
位置指令(POS_REF) → 位置PI → 速度指令(SPEED_REF) → 速度PI → 电流指令 → 电流PI → 电压
       ↑                                                                      │
       │                                                                      ↓
       └────────────────────────────────────────────────────────── 位置反馈 ← 电机
```

**实现代码**：

```c
if (motorflag_s.Conrorl_mode == 4)  // 位置闭环
{
    current_s.Idr = 0;
    current_s.Iqr = speed_s.SPEED_OUT;
}
```

**位置环PI调节**：

```c
void Position_PI(THETA_s *theta, SPEED_s *speed)
{
    // 位置误差计算（考虑360°环绕）
    theta->PositionErr = theta->PositionRef - theta->MotorTeta;

    // 角度误差处理（最短路径）
    if (theta->PositionErr > 180)
        theta->PositionErr -= 360;
    else if (theta->PositionErr < -180)
        theta->PositionErr += 360;

    // 位置PI输出作为速度指令
    speed->SPEED_REF = theta->PositionErr * theta->PositionKp + theta->PositionKi_out;

    // 积分项
    theta->PositionKi_out += theta->PositionErr * theta->PositionKi;
    theta->PositionKi_out = Limit_Sat(theta->PositionKi_out, 100, -100);

    // 速度限幅
    speed->SPEED_REF = Limit_Sat(speed->SPEED_REF, SPEED_LIM, -SPEED_LIM);
}
```

**参数配置**：

```c
#define POSITION_KP  2.0f    // 位置环比例系数
#define POSITION_KI  0.01f   // 位置环积分系数
```

### 3.6 模式5：无感速度闭环（Conrorl_mode = 5）

**控制目标**：无传感器速度控制

**闭环结构**：SMO观测器 + 速度环 + 电流环

```
速度指令 → 速度PI → 电流指令 → 电流PI → 电压 → SVPWM → 电机
       ↑                                                    │
       │                                                    ↓
       └──────────────────────────────────────────────── SMO观测器 ← 电流反馈
                                                        │
                                                        ↓
                                                    角度估算
```

**实现代码**：

```c
if (motorflag_s.Conrorl_mode == 5)  // 无感模式
{
    if (theta_s.flag == 1)  // IF启动阶段
    {
        current_s.Idr = paraset_s.openloop_Idref;
        current_s.Iqr = paraset_s.openloop_Iqref;
        speed_s.SPEED_REF = 0;
    }
    else if (theta_s.flag == 2)  // 混合过渡阶段
    {
        current_s.Idr = 0;
        current_s.Iqr = speed_s.SPEED_OUT;
    }
    else if (theta_s.flag == 3)  // SMO速度闭环
    {
        current_s.Idr = 0;
        current_s.Iqr = speed_s.SPEED_OUT;
    }

    Angle_SMOPare.Kslide = paraset_s.smo_kslide;
    Angle_SMOPare.Kslf = paraset_s.smo_kslf;
}
```

**无感启动阶段划分**：

| 阶段 | flag值 | 速度范围 | 控制策略 | 角度来源 |
|------|--------|---------|---------|---------|
| IF启动 | 1 | 0~250 RPM | 注入Id电流，开环角度递增 | 开环自增 |
| 混合过渡 | 2 | 250~300 RPM | 逐步切换到速度闭环 | SMO+补偿 |
| 速度闭环 | 3 | >300 RPM | 纯SMO控制 | SMO估算 |

### 3.7 闭环结构层级关系

```
┌─────────────────────────────────────────────────────────────────┐
│                      控制模式选择                                │
├─────────────────────────────────────────────────────────────────┤
│  Mode=1: 电压开环                                                │
│          └── Ud, Uq直接输出 → SVPWM → 电机                       │
├─────────────────────────────────────────────────────────────────┤
│  Mode=2: 电流闭环                                                │
│          └── Id,Iq指令 → 电流PI → Ud,Uq → SVPWM → 电机          │
├─────────────────────────────────────────────────────────────────┤
│  Mode=3: 速度闭环                                                │
│          └── 速度指令 → 速度PI → Iq指令 → 电流PI → Ud,Uq → ...  │
├─────────────────────────────────────────────────────────────────┤
│  Mode=4: 位置闭环                                                │
│          └── 位置指令 → 位置PI → 速度指令 → 速度PI → Iq → ...   │
├─────────────────────────────────────────────────────────────────┤
│  Mode=5: 无感闭环                                                │
│          └── 速度指令 → 速度PI → Iq指令 → 电流PI → SVPWM → 电机 │
│                                     ↑                           │
│                                     └── SMO观测器估算角度         │
└─────────────────────────────────────────────────────────────────┘
```

### 3.8 各模式对比总结

| 模式 | 控制目标 | 闭环层次 | 反馈来源 | 典型应用 |
|------|---------|---------|---------|---------|
| **1** | 电压指令 | 0层（开环） | 无 | 硬件测试 |
| **2** | dq电流 | 1层（电流环） | 电流采样 | 力矩控制 |
| **3** | 转速(r/min) | 2层（速度+电流） | 编码器/霍尔 | 速度控制 |
| **4** | 位置角度 | 3层（位置+速度+电流） | 编码器 | 位置控制 |
| **5** | 转速(r/min) | 2层（速度+电流） | SMO估算 | 无传感器控制 |

---

## 四、总结

### 核心技术要点

1. **ADC采样系统**
   - 同步采样避免开关噪声
   - 自动零漂校准消除系统误差
   - 三相电流不平衡补偿提高控制精度

2. **角度获取方式**
   - 开环角度用于启动阶段
   - 编码器提供高精度位置反馈
   - SMO滑模观测器实现无传感器控制

3. **控制模式设计**
   - 层次化PID设计（位置→速度→电流）
   - 模式灵活切换适应不同应用
   - 无感控制降低系统成本

### 代码位置索引

| 功能 | 文件 | 代码位置 |
|------|------|---------|
| ADC采样处理 | [motorlib.c](../Foc/motorlib.c) | Adcpro() |
| 角度处理 | [motorlib.c](../Foc/motorlib.c) | ThetaPro() |
| 编码器处理 | [motorlib.c](../Foc/motorlib.c) | coderpro() |
| 电流环PI | [motorlib.c](../Foc/motorlib.c) | runcur_process() |
| 速度环PI | [motorlib.c](../Foc/motorlib.c) | SpeedLoop() |
| 位置环PI | [motorlib.c](../Foc/motorlib.c) | PositionLoop() |
| SMO观测器 | [Sensorless_SMO.c](../Foc/Sensorless_SMO.c) | Angle_Cale() |
| 控制模式选择 | [motor.c](../Foc/motor.c) | motor_control() |

---

**文档版本**：V1.0  
**创建日期**：2024年1月  
**更新日期**：2026年5月13日
