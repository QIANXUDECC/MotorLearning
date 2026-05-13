/*
作者：拾小白电控
日期：2024.1.20
qq群：1群：860465413
备注：代码仅供学习参考，未经允许，禁止商用或售卖


功能：FOC电机库
*/

#include "motorlib.h"
#include "math.h"
#include "MotorConfig.h"
#include "Sensorless_SMO.h"
////变量结构体
EPWM_s epwm_s;
CURRENT_s current_s;
SPEED_s speed_s;
THETA_s theta_s;
ADCVALUE_s adcvalue_s;
MOTORFLAG_s motorflag_s;
PARASET_s paraset_s;
SCOPE_s scope_s;
POSITION_s position_s;
int16_t DATATABLE[30];
int16_t *data_Arr16[30];	  // 内部参数数组16位数据
float *fdata_Arr[30];		  // 内部参数数组32位浮点
int32_t *data_Arr32[30];	  // 内部参数数组32位整形
int32_t *i32OnlyRead_Arr[10]; // 监控组
float reserve = 0.0f;		  // 保留 reserve;
MOTOR_BIT16 Motor_Bit16;
MONITOR_s Monitor_s;

/*
转速环变量初始化
*/
void SpeedLoopInit(SPEED_s *speed)
{
	speed->speed_UP = 0.0;
	speed->speed_UI = 0.0;
	speed->Speed_OutPreSat = 0.0;
	speed->spdFbSum = 0.0;

	speed->e_speed = 0.0;
	speed->x_speed = 0.0;
	speed->SpeedMAX = SPEED_LIM;
	speed->SpeedoutMAX = CURRENT_LIM;
	speed->SPEED_OUT = 0.0;
	// speed->SpeedLoopPrescaler = 10;
	// speed->SpeedLoopCount = 0;
}

/*
电流环变量初始化
*/
void CurLoopInit(CURRENT_s *current)
{
	current->e_id = 0.0;
	current->x_id = 0.0;
	current->e_iq = 0.0;
	current->x_iq = 0.0;
	current->Idr = 0;
	current->Iqr = 0;
	current->Ud = 0;
	current->Uq = 0;
	current->Currentout_MAX = VOLTAGE_LIM;
}

// 正交编码器信号处理
void coderpro(THETA_s *theta, uint16_t coderCount)
{

	if (theta->motordir == 1) // 1编码器反向
	{
		theta->encoder = theta->OnePulseNum - coderCount;
	}
	else
	{
		theta->encoder = coderCount;
	}

	theta->OldTheta = theta->NewTheta;		   // 保留上一个机械角度脉冲
	theta->NewTheta = (int16_t)theta->encoder; // 更新编码器值
	if (theta->NewTheta >= theta->OnePulseNum)
		theta->NewTheta = 0;

	theta->realencoder = theta->encoder % theta->OnePulseNum; // 机械角度脉冲

	theta->MotorTeta = (int16_t)((int32_t)(theta->realencoder * 360) / (theta->OnePulseNum)); // 机械角度换算

	theta->encoderelc = theta->encoder % (theta->OnePulseNum / theta->MotorPn); // 电角度脉冲

	theta->Teta = (int16_t)((theta->encoderelc * 360) / (theta->OnePulseNum / theta->MotorPn)); // 4极对数  算电角度

	// theta->Teta += theta_s.PosInit;//;338;//25;
	if (theta->Teta >= 360) // 范围0-360
		theta->Teta -= 360;

	theta->ThetacountInc = theta->NewTheta - theta->OldTheta;
	if (theta->ThetacountInc > theta->OnePulseNum / 2) // 反转过零点
		theta->ThetacountInc = theta->NewTheta - theta->OnePulseNum - theta->OldTheta;
	if (theta->ThetacountInc < -theta->OnePulseNum / 2) // 正转过零点
		theta->ThetacountInc = theta->OnePulseNum - theta->OldTheta + theta->NewTheta;
	theta->ThetaSum += (float)theta->ThetacountInc;
}

// 位置环，Kp控制，累计位置
void PositionLoop(POSITION_s *position, SPEED_s *speed, THETA_s *theta)
{
	position->PostionSum -= (int32_t)theta->ThetacountInc;
	position->Postion += (float)(theta->ThetacountInc) * 360.0f / (float)theta->OnePulseNum;
	if (position->Postion > 65535.0f) // 最终给上位机的数据是16位的
		position->Postion -= 65535.0f;
	else if (position->Postion < -65535.0f)
		position->Postion += 65535.0f;
	if (speed->SpeedLoopCount == 0)
	{
		position->PostionOut = position->Kp * (position->PostionSum - 0);
		if (position->PostionOut > position->PostionOutMAX)
			position->PostionOut = position->PostionOutMAX;
		if (position->PostionOut < -position->PostionOutMAX)
			position->PostionOut = -position->PostionOutMAX;
		speed->SpeedTarget = position->PostionOut;
	}
}

// 速度环
void SpeedLoop(SPEED_s *speed, CURRENT_s *current, THETA_s *theta)
{
	// 低通滤波器，备用
	const float fc = 10000; // 截止频率
	const float Ts = 0.001; // 采样周期
	float b = 2 * 3.1415926f * fc * Ts;
	float alpha = b / (b + 1); // alpha
	float curlim;

	float SpeedAdd;

	// 有感和无感用两套参数
	if (motorflag_s.Conrorl_mode == 5)
	{
		SpeedAdd = 0.1f;
		speed->Kp = speed->Obs_Kp;
		speed->Ki = speed->Obs_Ki;
		curlim = CURRENT_LIM; // 防止无感下过流
	}
	else
	{
		SpeedAdd = 1.0f;
		speed->Kp = speed->Kp;
		speed->Ki = speed->Ki;
		curlim = CURRENT_LIM;
	}

	float SpeedAddUp = 0.5f;
	float SpeedAddDown = 1.0f;

	if(speed->SpeedRef < speed->SpeedTarget)
	{
		speed->SpeedRef += SpeedAddUp;
		if(speed->SpeedRef > speed->SpeedTarget)
			speed->SpeedRef = speed->SpeedTarget;
	}
	else
	{
		speed->SpeedRef -= SpeedAddDown;
		if(speed->SpeedRef < speed->SpeedTarget)
			speed->SpeedRef = speed->SpeedTarget;
	}


	// 1/10倍电流环频率
	if (speed->SpeedLoopCount++ >= speed->SpeedLoopPrescaler)
	{
		speed->SpeedLoopCount = 0;
		if (motorflag_s.Conrorl_mode == 5) // 无感下使用估算转速
		{

			speed->SpeedRef = Limit(speed->SpeedRef, SPEED_LIM);										   // 转速给定限幅
			speed->speedminus = speed->speedminus + alpha * (Speed_estPare.Speed_RPM - speed->speedminus); // 转速来自观测器
			if (speed->speedminus < 50.0f)
				speed->speedminus = 50.0f;
			speed->speedshow = speed->speedminus;
		}
		else
		{
			speed->SpeedRef = Limit(speed->SpeedRef, SPEED_LIM);								 // 转速给定限幅
			speed->speedminus = theta->ThetaSum * (60.0f * 1000.0f / (float)theta->OnePulseNum); // 不使用滤波    n = x/2000*1000

			// 低通滤波器工程算法
			// y(n)=y(n-1)+alpha*(x(n)-y(n-1))

			speed->speedshow = speed->speedshow + alpha * ((theta->ThetaSum * (60.0f * 1000.0f / (float)theta->OnePulseNum)) - speed->speedshow);
		}
		theta->ThetaSum = 0;

		// speed = (float)ThetaSum*15;     //真实机械角速度  r/min
		// speedred = speed*0.10472*4;  //电气角速度 rad/s

		// PI+限幅控制,id=0控制输出值给到iq
		speed->e_speed = speed->SpeedRef - speed->speedminus;
		speed->speed_UP = speed->Kp * speed->e_speed;
		speed->speed_UI = speed->speed_UI + speed->Ki * speed->e_speed;
		speed->speed_UI = Limit(speed->speed_UI, curlim);
		speed->Speed_OutPreSat = speed->speed_UP + speed->speed_UI;

		speed->SPEED_OUT = Limit(speed->Speed_OutPreSat, curlim);
	}
}

// 电机角度/频率 处理函数 0 角度自增  1编码器闭环 2固定位置
void ThetaPro(THETA_s *theta, MOTORFLAG_s *mode)
{
	// 转速/min->电角度/s     系数=1/60*2*pi *pn
	if (mode->Theta_mode == 0)
	{
		// 位置开环给定角度信号生成
		// theta->TetaInc+=theta->TetaIncInc;  //加速度-->速度
		if (theta->TetaInc > 100.0f) // 开环最大转速
		{
			theta->TetaInc = 100.0f;
		}
		theta->Tetatmp += (theta->TetaInc * 0.036f);
		// 电角度限定
		if (theta->Tetatmp > 360)
		{
			theta->Tetatmp -= 360;
		}
		else if (theta->Tetatmp < 0)
		{
			theta->Tetatmp += 360;
		}
		theta->TetaRef = (uint16_t)theta->Tetatmp;
	}
	else if (mode->Theta_mode == 1) // 编码器闭环
	{
		theta->Teta = theta->Teta - theta->PosInit;
		if (theta->Teta > 360)
		{
			theta->Teta -= 360;
		}
		else if (theta->Teta < 0)
		{
			theta->Teta += 360;
		}
		theta->TetaRef = theta->Teta;
	}
	else if (mode->Theta_mode == 2) // 固定角度
	{
		theta->TetaRef = theta->CountTeta;
	}

	if (mode->Conrorl_mode == 5) // 无感控制,后续用PMSM就不用搞这么多补偿判断了
	{
		static float Speedlast;

		theta->flag = 1;
		if (Speed_estPare.Speed_RPM2 > 250) // 滞环选择
		{
			if (Speed_estPare.Speed_RPM2 < 300) // 切电流环
			{
				paraset_s.openloop_Iqref = 0.5f;
				theta->TetaRef = theta->TetaSMO;
				theta->flag = 2;
			}
			else // 切速度环
			{
				theta->TetaRef = theta->TetaSMO;
				theta->flag = 3;
			}
			// 转速估算发散，自动复位
			if (fabs(Speed_estPare.Speed_RPM2 - Speedlast) > 50.0f)
			{
				theta->flag = 1;
				theta->TetaInc = 0;
			}
		}
		else if (Speed_estPare.Speed_RPM2 < 300) // 进行IF启动
		{
			paraset_s.openloop_Iqref = 1.0f;

			theta->TetaInc += 0.02f; // 随便给一个速度拉升
			if (theta->TetaInc > 35.0f)
				theta->TetaInc = 35.0f;

			theta->Tetatmp += (theta->TetaInc * 0.036f);
			// 电角度限定
			if (theta->Tetatmp > 360)
			{
				theta->Tetatmp -= 360;
			}
			else if (theta->Tetatmp < 0)
			{
				theta->Tetatmp += 360;
			}
			theta->TetaRef = (uint16_t)theta->Tetatmp;
		}

		if (Motor_Bit16.bits.RUN != 1) // 待机下不要给角度
		{
			theta->TetaRef = 0;
			theta->TetaInc = 0;
			theta->flag = 1;
		}

		Speedlast = Speed_estPare.Speed_RPM2;
	}
	else if (theta->flag != 0)
	{
		theta->flag = 0;
		paraset_s.openloop_Iqref = 0.5f;
	}
	// 三角函数换算
	theta->Sin_Teta = SinTetaCalculate(theta->TetaRef);
	theta->Cos_Teta = CosTetaCalculate(theta->TetaRef);
}

extern uint16_t aADCxConvertedData[4];
// 电流采样信号处理
// 输入为ADC电流12位原始值,如果不是12位请转成12位
void Adcpro(ADCVALUE_s *adcvalue, CURRENT_s *current, MOTORFLAG_s *motorflag, uint16_t *Iraw)
{
	adcvalue->Iu_Sample = -Iraw[0]; // 反向，一般电流取流入电机方向为正
	adcvalue->Iv_Sample = -Iraw[1];
	adcvalue->Iw_Sample = -Iraw[2];
	// ADC系数转化
	adcvalue->Iu = ((float)adcvalue->Iu_Sample * 3.3f / 4096 - adcvalue->adc_refer) * adcvalue->gain_cur;
	adcvalue->Iv = ((float)adcvalue->Iv_Sample * 3.3f / 4096 - adcvalue->adc_refer) * adcvalue->gain_cur;
	adcvalue->Iw = ((float)adcvalue->Iw_Sample * 3.3f / 4096 - adcvalue->adc_refer) * adcvalue->gain_cur;
	// adcvalue->Udc = ((float)Iraw[3]*3.3f/4096.0f-adcvalue->adc_refer)*adcvalue->gain_udc;
	adcvalue->Udc = ((float)Iraw[3] * 3.3f / 4096.0f - 0) * adcvalue->gain_udc;
	// 母线异常判断
	if ((adcvalue->Udc > DCBUSVALUE - 5.0f) && (adcvalue->Udc < DCBUSVALUE + 5.0f)) // 母线电压正常
	{
		motorflag->ERR_UDC = 0;
		if (adcvalue->adcstateflag < 2) // 上电后adc电流校准未开启
		{
			// 电流校准零偏补偿
			if (adcvalue->ADCcount < 5000) // 5000次=0.5s
			{
				adcvalue->adcstateflag = 1;
				adcvalue->ADCcount++;
				adcvalue->Iu_Sample_offer = adcvalue->Iu_Sample_offer * 0.998f + (float)adcvalue->Iu * 0.002f;
				adcvalue->Iv_Sample_offer = adcvalue->Iv_Sample_offer * 0.998f + (float)adcvalue->Iv * 0.002f;
				adcvalue->Iw_Sample_offer = adcvalue->Iw_Sample_offer * 0.998f + (float)adcvalue->Iw * 0.002f;
			}
			else
			{
				adcvalue->adcstateflag = 2; // 校准完成
				adcvalue->ADCcount = 0;
			}
		}

		if (adcvalue->adcstateflag == 2) // adc电流校准完成
		{
			current->Ia = (adcvalue->Iu - adcvalue->Iu_Sample_offer);
			current->Ib = (adcvalue->Iv - adcvalue->Iv_Sample_offer);
			current->Ic = (adcvalue->Iw - adcvalue->Iw_Sample_offer);
			motorflag->Motor_EnableFlag = 1; // 母线电流正常，电机使能位打开

			// 判断是否过流保护
			if ((((current->Ia > CURRENTOVER_MAX) || (current->Ia < -CURRENTOVER_MAX)) || ((current->Ib > CURRENTOVER_MAX) || (current->Ib < -CURRENTOVER_MAX)) || ((current->Ic > CURRENTOVER_MAX) || (current->Ic < -CURRENTOVER_MAX))))
			{
				motorflag->ERR_OVERCURRENT = 1;
			}
			if (fabs(adcvalue->currentoffera - 1.0f) < 0.5f && fabs(adcvalue->currentofferb - 1.0f) < 0.5f && fabs(adcvalue->currentofferc - 1.0f) < 0.5f && adcvalue->currentofferEN == 1)
			{
				current->Ia = current->Ia * adcvalue->currentoffera; // 软件校正
				current->Ib = current->Ib * adcvalue->currentofferb;
				current->Ic = current->Ic * adcvalue->currentofferc;
			}
		}
		else
		{
			current->Ia = adcvalue->Iu;
			current->Ib = adcvalue->Iv;
			current->Ic = adcvalue->Iw;
		}
	}
	else // 母线电压异常
	{
		current->Ia = adcvalue->Iu;
		current->Ib = adcvalue->Iv;
		current->Ic = adcvalue->Iw;
		adcvalue->ADCcount = 0;
		adcvalue->adcstateflag = 0;
		motorflag->Motor_EnableFlag = 0;
		motorflag->ERR_UDC = 1;
	}
}

/*
函数名：park变换
输入：ia,ib,ic,Sin_Teta,Cos_Teta
输出：id,iq
备注：可以选择是否滤波
*/
void Park_abctodq(CURRENT_s *current, THETA_s *theta)
{
	// current->Ic = -current->Ia-current->Ib;
	//-------------------------------------------------
	// Ialfa = Ia
	// Ibeta = (2*Ib + Ia) / sqrt(3)    //1/sqrt(3)=0.57735
	//-------------------------------------------------
	current->Ialfa = current->Ia;
	current->Ibeta = (current->Ia + current->Ib * 2.0f) * 0.57735f;
	//-------------------------------------------------
	// Id =  Ialfa * cos(Teta) + Ibeta * sin(Teta)
	// Iq = -Ialfa * sin(Teta) + Ibeta * cos(Teta)
	//-------------------------------------------------
	current->Id = ((float)theta->Cos_Teta * current->Ialfa + (float)theta->Sin_Teta * current->Ibeta) * 0.00024414f;
	current->Iq = (current->Ibeta * (float)theta->Cos_Teta - current->Ialfa * (float)theta->Sin_Teta) * 0.00024414f;
}

/*
函数名：电流环处理函数
输入:电流里的Idr，Iqr
输出:电压值给SVPWM
*/
void runcur_process(CURRENT_s *current, EPWM_s *epwm)
{
	// pi控制+限幅
	// iq处理
	current->e_iq = current->Iqr - current->Iq;					 // 电流iq误差
	current->x_iq = current->x_iq + current->e_iq * current->Ki; // 误差积分累计
	// 积分限幅
	current->x_iq = Limit(current->x_iq, current->Currentout_MAX);
	current->Uqbuf = current->x_iq + current->e_iq * current->Kp; // q轴电压原始值
	// 电压输出
	current->Uq = Limit(current->Uqbuf, current->Currentout_MAX);

	// id处理
	current->e_id = current->Idr - current->Id;					 // 电流id误差
	current->x_id = current->x_id + current->e_id * current->Ki; // 误差积分累计
	// 积分限幅
	current->x_id = Limit(current->x_id, current->Currentout_MAX);
	current->Udbuf = current->x_id + current->e_id * current->Kp; // q轴电压原始值
	// 电压输出
	current->Ud = Limit(current->Udbuf, current->Currentout_MAX);
}
/*
SVPWM定点算法
输入ud,uq,Sin_Teta,Cos_Teta,SVPWM计数最大值
*/
void SVPWM(CURRENT_s *current, THETA_s *theta, uint16_t pwm_count, EPWM_s *epwm)
{
	// 按照载波频率定标，提前转为定点算法
	epwm->Udref = (int32_t)(current->Ud * pwm_count * 1.73205f / DCBUSVALUE);
	epwm->Uqref = (int32_t)(current->Uq * pwm_count * 1.73205f / DCBUSVALUE);

	// alfa beta
	epwm->Ualfa = (((int32_t)epwm->Udref) * theta->Cos_Teta - ((int32_t)epwm->Uqref) * theta->Sin_Teta) >> 12;
	epwm->Ubeta = (((int32_t)epwm->Udref) * theta->Sin_Teta + ((int32_t)epwm->Uqref) * theta->Cos_Teta) >> 12;
	// 中间变量
	epwm->Va = epwm->Ubeta;
	epwm->Vb = -(epwm->Ubeta >> 1) + ((((int32_t)epwm->Ualfa) * 0x0ddb) >> 12);
	epwm->Vc = -(epwm->Ubeta >> 1) - ((((int32_t)epwm->Ualfa) * 0x0ddb) >> 12);
	// 扇区判断
	if (epwm->Va > 0)
		epwm->A = 1;
	else
		epwm->A = 0;
	if (epwm->Vb > 0)
		epwm->B = 1;
	else
		epwm->B = 0;
	if (epwm->Vc > 0)
		epwm->C = 1;
	else
		epwm->C = 0;
	epwm->sector = epwm->A + (epwm->B << 1) + (epwm->C << 2);
	// 作用时间计算
	switch (epwm->sector)
	{
	case 1:
		epwm->t1 = -epwm->Vb;
		epwm->t2 = -epwm->Vc;
		break;
	case 2:
		epwm->t1 = -epwm->Vc;
		epwm->t2 = -epwm->Va;
		break;
	case 3:
		epwm->t1 = epwm->Vb;
		epwm->t2 = epwm->Va;
		break;
	case 4:
		epwm->t1 = -epwm->Va;
		epwm->t2 = -epwm->Vb;
		break;
	case 5:
		epwm->t1 = epwm->Va;
		epwm->t2 = epwm->Vc;
		break;
	case 6:
		epwm->t1 = epwm->Vc;
		epwm->t2 = epwm->Vb;
		break;
	default:
		break;
	}
	// 过调制限定
	if ((epwm->t1 + epwm->t2) > pwm_count)
	{
		epwm->cTime1 = (int16_t)((pwm_count * ((int32_t)epwm->t1)) / (epwm->t1 + epwm->t2));
		epwm->cTime2 = (int16_t)((pwm_count * ((int32_t)epwm->t2)) / (epwm->t1 + epwm->t2));
		epwm->t1 = epwm->cTime1;
		epwm->t2 = epwm->cTime2;
	}
	epwm->taon = (pwm_count - epwm->t1 - epwm->t2) >> 1;
	epwm->tbon = epwm->taon + epwm->t1;
	epwm->tcon = epwm->tbon + epwm->t2;

	// 占空比计算
	switch (epwm->sector)
	{
	case 0:
		epwm->epwm1 = PWM_PERIOD / 2;
		epwm->epwm2 = PWM_PERIOD / 2;
		epwm->epwm3 = PWM_PERIOD / 2;
		break;
	case 1:
		epwm->epwm1 = epwm->tbon;
		epwm->epwm2 = epwm->taon;
		epwm->epwm3 = epwm->tcon;
		break;
	case 2:
		epwm->epwm1 = epwm->taon;
		epwm->epwm2 = epwm->tcon;
		epwm->epwm3 = epwm->tbon;
		break;
	case 3:
		epwm->epwm1 = epwm->taon;
		epwm->epwm2 = epwm->tbon;
		epwm->epwm3 = epwm->tcon;
		break;
	case 4:
		epwm->epwm1 = epwm->tcon;
		epwm->epwm2 = epwm->tbon;
		epwm->epwm3 = epwm->taon;
		break;
	case 5:
		epwm->epwm1 = epwm->tcon;
		epwm->epwm2 = epwm->taon;
		epwm->epwm3 = epwm->tbon;
		break;
	case 6:
		epwm->epwm1 = epwm->tbon;
		epwm->epwm2 = epwm->tcon;
		epwm->epwm3 = epwm->taon;
		break;
	default:
		break;
	}
}

// 正负上下限幅
float Limit(float value, float lim)
{
	float tmp = value;
	if (fabsf(tmp) > lim)
	{
		if (tmp > 0)
		{
			tmp = lim;
		}
		else
		{
			tmp = -lim;
		}
	}
	return tmp;
}

// 按键1使能函数接口
void key1FlagSet(uint16_t keyvalue)
{
	motorflag_s.Key_EnPWM = keyvalue;
}

void datasave(void)
{
	// 第一组  0-9   电机状态
	DATATABLE[0] = (int16_t)theta_s.TetaRef;		 // 00给定电角度
	DATATABLE[1] = (int16_t)theta_s.Teta;			 // 01实际电角度
	DATATABLE[2] = (int16_t)epwm_s.epwm1;			 // 02马鞍波
	DATATABLE[3] = reserve;							 // 03保留
	DATATABLE[4] = reserve;							 // 04保留
	DATATABLE[5] = (int16_t)(adcvalue_s.Udc * 1000); // 05Udc
	DATATABLE[6] = (int16_t)(speed_s.speedshow);	 // 06转速/分
	DATATABLE[7] = (int16_t)(position_s.Postion);	 // 07绝对电角度
	DATATABLE[8] = (int16_t)(theta_s.MotorTeta);	 // 08机械角度
	DATATABLE[9] = reserve;							 // 09保留
	//
	// 第二组  10-19   观测变量
	DATATABLE[10] = (int16_t)(current_s.Ia * 1000);	   // 10Ia
	DATATABLE[11] = (int16_t)(current_s.Ib * 1000);	   // 11Ib
	DATATABLE[12] = (int16_t)(current_s.Ic * 1000);	   // 12Ic
	DATATABLE[13] = (int16_t)(current_s.Ialfa * 1000); // 13Ialfa
	DATATABLE[14] = (int16_t)(current_s.Ibeta * 1000); // 14Ibeta
	DATATABLE[15] = (int16_t)(current_s.Id * 1000);	   // 15Id
	DATATABLE[16] = (int16_t)(current_s.Iq * 1000);	   // 16Iq
	DATATABLE[17] = (int16_t)(current_s.Ud * 1000);	   // 17Ud
	DATATABLE[18] = (int16_t)(current_s.Uq * 1000);	   // 18Uq
	DATATABLE[19] = reserve;						   // 19保留
	//
	// 第三组  自定义变量
	DATATABLE[20] = (int16_t)((float)IQAtan_Pare.Alpha * 1000 / 65535); // 21test2		  反电动势α
	DATATABLE[21] = (int16_t)((float)IQAtan_Pare.Beta * 1000 / 65535);	// 22test3			反电动势β
	DATATABLE[22] = (int16_t)(theta_s.TetaSMO);							// 23test4			观测位置
	DATATABLE[23] = (int16_t)(Speed_estPare.Speed_RPM);					// 23test4		  观测转速
	DATATABLE[24] = (int16_t)(Speed_estPare.Speed_RPM2);				// 24test5
	DATATABLE[25] = reserve;
	DATATABLE[26] = reserve;
	DATATABLE[27] = reserve;
	DATATABLE[28] = reserve;
	DATATABLE[29] = reserve;
}

// 监控数据，计算电机当前相关参数和指标
void MonitorPro(void)
{
	float fc = 10;			// 截止频率
	const float Ts = 0.001; // 采样周期
	float b = 2 * 3.1415926f * fc * Ts;
	float alpha = b / (b + 1); // alpha
	if (motorflag_s.Motor_EnableFlag == 1)
	{
		Monitor_s.Uref = Monitor_s.Uref + alpha * ((int32_t)(sqrtf(current_s.Ud * current_s.Ud + current_s.Uq * current_s.Uq) * 1000) - Monitor_s.Uref);
		Monitor_s.Iact = Monitor_s.Iact + alpha * ((int32_t)(sqrtf(current_s.Id * current_s.Id + current_s.Iq * current_s.Iq) * 1000) - Monitor_s.Iact);
	}
	else
	{
		Monitor_s.Uref = 0;
		Monitor_s.Iact = 0;
	}
	// Monitor_s.Speedact = Monitor_s.Speedact + alpha*((int32_t)((int32_t)(speed_s.speedminus*1000)-Monitor_s.Speedact));
	Monitor_s.Speedact = Monitor_s.Speedact + alpha * ((int32_t)((int32_t)(speed_s.speedshow * 1000) - Monitor_s.Speedact));
	Monitor_s.Motor_Bit16 = (int32_t)Motor_Bit16.u16_Motorflag;
	Monitor_s.Conrorl_mode = (int32_t)motorflag_s.Conrorl_mode;
	Monitor_s.Theta_mode = (int32_t)motorflag_s.Theta_mode;
}
