/*
作者：拾小白电控
日期：2023.8.5
qq群：1群：860465413
备注：代码仅供学习参考，未经允许，禁止商用或售卖


功能：FOC主中断执行
*/

#include "motor.h"
#include "math.h"
#include "user.h"
#include "motorlib.h"
#include "Table.h"
#include "MotorConfig.h"
#include "flash.h"
#include "Sensorless_SMO.h"
// #include "Comm_usart.h"

// 注入通道采样值
extern uint16_t aADCxINJConvertedData[4];
extern uint16_t aADCxConvertedData[4];
extern uint16_t encoderCount;
extern int16_t Led1Value;

// flash保存相关电机参数
uint32_t flashpage1datasave[16] = {0};
uint32_t flashpage1dataread[16] = {0};

void motor_control(void)
{
	// ADC采样处理
	Adcpro(&adcvalue_s, &current_s, &motorflag_s, aADCxINJConvertedData);

	if (theta_s.flag != 0)
	{
		// 无感部分，用上一拍的值
		Angle_SMOPare.Ialpha = current_s.Ialfa;
		Angle_SMOPare.Ibeta = current_s.Ibeta;
		Angle_SMOPare.Valpha = (float)epwm_s.Ualfa * DCBUSVALUE / PWM_PERIOD / 1.73205f; // 转换成真实电压
		Angle_SMOPare.Vbeta = (float)epwm_s.Ubeta * DCBUSVALUE / PWM_PERIOD / 1.73205f;
		Angle_Cale((p_Angle_SMO)&Angle_SMOPare);

		IQAtan_Pare.Alpha = (int32_t)(-Angle_SMOPare.Ealpha * 65535);	  // 转Q15
		IQAtan_Pare.Beta = (int32_t)(Angle_SMOPare.Ebeta * 65535);		  //
		IQAtan_Cale(&IQAtan_Pare);										  // 反正切查表计算
		theta_s.TetaSMO = (int16_t)(IQAtan_Pare.IQAngle_JZ * 360 / 1024); // 转换到0-360后进行补偿
		if (theta_s.TetaSMO > 360)
			theta_s.TetaSMO -= 360;
	}

	// 编码器处理
	coderpro(&theta_s, encoderCount);
	// 电角度信号处理
	ThetaPro(&theta_s, &motorflag_s);
	// park变换
	Park_abctodq(&current_s, &theta_s);
	//
	//	=================位置环控制===================================
	//	=================速度环控制===================================
	if (motorflag_s.Conrorl_mode == 4) // 位置环
	{
		if (paraset_s.Postionloopref != 0)
		{
			position_s.PostionSum = (int32_t)paraset_s.Postionloopref * ((int32_t)theta_s.OnePulseNum / 360.0);
			paraset_s.Postionloopref = 0;
		}
		PositionLoop(&position_s, &speed_s, &theta_s);
	}
	else if (motorflag_s.Conrorl_mode == 3 || motorflag_s.Conrorl_mode == 5) // 速度环控制
	{
		speed_s.SpeedTarget = paraset_s.speedloopref;
	}
	else
	{
		speed_s.SpeedTarget = 0;
	}
	// 速度环，执行频率=1/10电流环频率
	SpeedLoop(&speed_s, &current_s, &theta_s);
	//=================电流环控制===================================
	// 根据当前的控制方式进行电流环处理
	if (motorflag_s.Conrorl_mode == 1) // 电压给定
	{
		SpeedLoopInit(&speed_s);
		CurLoopInit(&current_s);
		current_s.Ud = paraset_s.openloop_Udref;
		current_s.Uq = paraset_s.openloop_Uqref;
	}
	else if (motorflag_s.Conrorl_mode == 2) // 电流给定
	{
		current_s.Idr = paraset_s.openloop_Idref;
		current_s.Iqr = paraset_s.openloop_Iqref;
		runcur_process(&current_s, &epwm_s);

		SpeedLoopInit(&speed_s);
	}
	else if (motorflag_s.Conrorl_mode == 3 || motorflag_s.Conrorl_mode == 4) // 转矩给定
	{
		current_s.Idr = 0;
		current_s.Iqr = speed_s.SPEED_OUT;
		runcur_process(&current_s, &epwm_s);
	}
	else if (motorflag_s.Conrorl_mode == 5) // 无感控制
	{
		if (theta_s.flag == 1 || theta_s.flag == 2) // IF强拖状态
		{
			current_s.Idr = paraset_s.openloop_Idref;
			current_s.Iqr = paraset_s.openloop_Iqref;
			runcur_process(&current_s, &epwm_s);
			SpeedLoopInit(&speed_s);
		}
		else if (theta_s.flag == 3) // 速度闭环状态
		{
			current_s.Idr = 0;
			current_s.Iqr = speed_s.SPEED_OUT;
			runcur_process(&current_s, &epwm_s);
		}
	}
	else
	{
		// 找零点定位状态，默认直接给定电压定位
	}
	//=================SVPWM调制===================================
	SVPWM(&current_s, &theta_s, PWM_PERIOD, &epwm_s);

	// motorflag_s.Motor_EnableFlag = 1;
	if ((motorflag_s.Key_EnPWM == 1 || motorflag_s.Zero_EnPWM) && (motorflag_s.Motor_EnableFlag == 1)) // 判断是否打开PWM
	{
		if (motorflag_s.ERR_OVERCURRENT == 0)
		{
			PWM_Enable();
			Set_SVPWM_Compare(epwm_s.epwm1, epwm_s.epwm2, epwm_s.epwm3);
			Motor_Bit16.bits.RUN = 1;
		}
		else // 软件过流封驱动
		{
			PWM_DisEnable();
		}
	}
	else if (adcvalue_s.adcstateflag == 1) // 上电零漂校准，给0电压
	{
		PWM_Enable();
		Set_SVPWM_Compare(4200, 4200, 4200);
		Motor_Bit16.bits.RUN = 1;
	}
	else
	{
		// 积分清零
		PWM_DisEnable();
		// 电流环变量初始化
		// 转速环变量初始化
		CurLoopInit(&current_s);
		SpeedLoopInit(&speed_s);
		Set_SVPWM_Compare(4200, 4200, 4200);
		Motor_Bit16.bits.RUN = 0;
	}
}

// 电机参数初始化
void parameter_init(void)
{
	theta_s.MotorPn = Pn;				 // 磁极对数
	theta_s.OnePulseNum = PluseNum;		 // 单圈线数
	motorflag_s.Conrorl_mode = CTR_MODE; // 控制方式  1电压给定 2电流给定 3速度环 4位置环
	motorflag_s.Theta_mode = TETA_MODE;	 // 角度给定  0 角度自增  1编码器闭环 2固定位置
	motorflag_s.Encorder = ENCODER;
	motorflag_s.version = VERSION; // 版本号
	// theta_s.PosInit = 300;//200   			//位置补偿
	theta_s.CountTeta = 0;			 // 固定绝对角度
	adcvalue_s.adc_refer = ADCREFER; // 采样系数

	adcvalue_s.gain_cur = ADCCURGAIN; // 采样增益
	adcvalue_s.gain_udc = ADCUDCGAIN;
	adcvalue_s.Udc = DCBUSVALUE;
	adcvalue_s.currentofferEN = CURRENTOFFEREN;
	adcvalue_s.currentoffera = 1;
	adcvalue_s.currentofferb = 1;
	adcvalue_s.currentofferc = 1;

	//	//电流环参数初值
	current_s.Kp = CURRENT_Kp; //       0.00059*300*2*pi = 1.13    --->1130
	current_s.Ki = CURRENT_Ki; //      1*300*2*pi/10000 = 0.1884   ---188

	paraset_s.openloop_Idref = 0.0f;
	paraset_s.openloop_Iqref = 0.5f;
	//
	paraset_s.openloop_Udref = 0.0f;
	paraset_s.openloop_Uqref = 0.5f;
	//
	theta_s.TetaInc = 7.0f; // 给定频率模式初值

	position_s.Kp = POSITION_Kp;
	position_s.PostionOutMAX = POSITIONOUTMAX;

	//
	paraset_s.speedloopref = 500;
	speed_s.Kp = SPEED_Kp;
	speed_s.Ki = SPEED_Ki;
	speed_s.Obs_Kp = Obs_SPEED_Kp;
	speed_s.Obs_Ki = Obs_SPEED_Ki;
	speed_s.SpeedLoopPrescaler = 9;

	// 无感参数初始化
	SMO_Pare_init();

	scope_s.scope1 = SCOPEGROUP1; // 示波器通道初始化
	scope_s.scope2 = SCOPEGROUP2;
	scope_s.scope3 = SCOPEGROUP3;
	scope_s.scope4 = SCOPEGROUP4;

	// 16位整数组地址绑定
	data_Arr16[0] = &Led1Value;				   // 按键使能
	data_Arr16[1] = &motorflag_s.Conrorl_mode; // 控制模式
	data_Arr16[2] = &motorflag_s.Theta_mode;   // 角度模式
	data_Arr16[3] = &motorflag_s.ZeroFlag;
	data_Arr16[5] = &motorflag_s.ResetErr; // 故障复位标志

	data_Arr16[10] = &theta_s.CountTeta;

	data_Arr16[20] = &scope_s.scope1;
	data_Arr16[21] = &scope_s.scope2;
	data_Arr16[22] = &scope_s.scope3;
	data_Arr16[23] = &scope_s.scope4;

	data_Arr16[29] = &motorflag_s.version;
	// 32位浮点数据地址绑定
	fdata_Arr[0] = (float *)&paraset_s.openloop_Udref; // 开环ud
	fdata_Arr[1] = (float *)&paraset_s.openloop_Uqref; // 开环uq
	fdata_Arr[2] = (float *)&paraset_s.openloop_Idref; // 开环id
	fdata_Arr[3] = (float *)&paraset_s.openloop_Iqref; // 开环iq
	fdata_Arr[4] = (float *)&paraset_s.speedloopref;   // 给定转速

	fdata_Arr[10] = &theta_s.TetaInc;

	fdata_Arr[20] = (float *)&current_s.Kp;
	fdata_Arr[21] = (float *)&current_s.Ki;
	fdata_Arr[22] = (float *)&speed_s.Kp;
	fdata_Arr[23] = (float *)&speed_s.Ki;
	fdata_Arr[24] = (float *)&position_s.Kp;

	// 32位整数组地址绑定
	data_Arr32[0] = (int32_t *)&paraset_s.Postionloopref; // 给定位置

	// 只读数据绑定,转化位32位整数给上位机显示
	i32OnlyRead_Arr[0] = (int32_t *)&Monitor_s.Motor_Bit16;
	i32OnlyRead_Arr[1] = (int32_t *)&Monitor_s.Conrorl_mode;
	i32OnlyRead_Arr[2] = (int32_t *)&Monitor_s.Theta_mode;
	i32OnlyRead_Arr[3] = (int32_t *)&Monitor_s.Uref;
	i32OnlyRead_Arr[4] = (int32_t *)&Monitor_s.Iact;
	i32OnlyRead_Arr[5] = (int32_t *)&Monitor_s.Speedact;

	// 读flash 避免重新上电反复对齐零位
	Flash_Page1_Read((uint32_t *)flashpage1dataread);
	if (flashpage1dataread[FLASH_ZEROFLAG] == 2)
	{
		theta_s.PosInit = (uint16_t)flashpage1dataread[FLASH_POSINIT];
		theta_s.motordir = (uint16_t)flashpage1dataread[FLASH_MOTORDIR];
		adcvalue_s.currentoffera = HEX_float(flashpage1dataread[FLASH_OFFERIU]);
		adcvalue_s.currentofferb = HEX_float(flashpage1dataread[FLASH_OFFERIV]);
		adcvalue_s.currentofferc = HEX_float(flashpage1dataread[FLASH_OFFERIW]);
	}
}

// 零位对齐+电流校正
void ZeroFind(void)
{
	static int Zerotime;
	static int16_t halfcountflag;
	static float Iasum, Ibsum, Icsum;
	static int savestate1, savestate2;
	static uint8_t dir;
	if (motorflag_s.ZeroFlag != 0) //
	{
		Zerotime++; // 10ms进行计数
		if (Zerotime == 1)
		{
			savestate1 = motorflag_s.Theta_mode;
			savestate2 = motorflag_s.Conrorl_mode;
		}

		motorflag_s.Conrorl_mode = 0; // 零点对齐控制
		motorflag_s.Theta_mode = 2;	  // 固定角度进行定位

		motorflag_s.Zero_EnPWM = 1; // 打开PWM开关
		if (Zerotime < 50)
		{
			theta_s.CountTeta = 0; // 固定0角度
			current_s.Ud = 1.0f;
			current_s.Uq = 0.0f;
		}
		else if (Zerotime == 50) // 一次定位完成
		{
			theta_s.PosInit = theta_s.Teta;
		}
		else if (Zerotime < 180 + 50) // 该阶段耗时1.8s,电机开环旋转一周,判断编码器方向,此处逻辑写的极不规范不要模仿哈
		{
			current_s.Ud = 0.5f;
			current_s.Uq = 0.0f;
			theta_s.CountTeta += (2 * Pn);
			if (theta_s.CountTeta > 360)
				theta_s.CountTeta -= 360;

			if (theta_s.Teta > 150 && theta_s.Teta < 210 && halfcountflag == 0)
				halfcountflag = theta_s.Teta; // 标记此时位置
			if (halfcountflag != 0 && (halfcountflag != theta_s.Teta) && (halfcountflag != 5555))
			{
				if (halfcountflag > theta_s.Teta)
				{
					dir = 1;
				}
				else
				{
					dir = 0;
				}
				halfcountflag = 5555; // 判定完成
			}
			Iasum += fabs(current_s.Ia);
			Ibsum += fabs(current_s.Ib);
			Icsum += fabs(current_s.Ic);
		}
		else if (Zerotime == 180 + 50)
		{
			if (dir == 1)
				theta_s.motordir = theta_s.motordir == 0 ? 1 : 0;
			flashpage1datasave[FLASH_MOTORDIR] = theta_s.motordir;
			// 电流校正
			float average = (Iasum + Ibsum + Icsum) / 3;
			adcvalue_s.currentoffera = (average / Iasum);
			adcvalue_s.currentofferb = (average / Ibsum);
			adcvalue_s.currentofferc = (average / Icsum);
			flashpage1datasave[FLASH_OFFERIU] = Float_HEX(adcvalue_s.currentoffera);
			flashpage1datasave[FLASH_OFFERIV] = Float_HEX(adcvalue_s.currentofferb);
			flashpage1datasave[FLASH_OFFERIW] = Float_HEX(adcvalue_s.currentofferc);
			Iasum = 0;
			Ibsum = 0;
			Icsum = 0;
		}
		else if (Zerotime < 180 + 50 + 50)
		{
			theta_s.CountTeta = 0; // 固定0角度
			current_s.Ud = 1.0f;
			current_s.Uq = 0.0f;
		}
		else if (Zerotime == 180 + 50 + 50) // 二次定位完成
		{
			motorflag_s.ZeroFlag = 2; // 表示零点已对齐过
			theta_s.PosInit = theta_s.Teta;
			// 保存到flash数组
			flashpage1datasave[FLASH_ZEROFLAG] = motorflag_s.ZeroFlag;
			flashpage1datasave[FLASH_POSINIT] = theta_s.PosInit;
			// Flash_Save64(DATAADDR1,motorflag_s.ZeroFlag,theta_s.PosInit);
		}
		else if (Zerotime < 180 + 60 + 50)
		{
			// wait
		}
		else
		{
			Flash_Page1_Save((uint32_t *)flashpage1datasave);
			motorflag_s.ZeroFlag = 0;
			motorflag_s.Zero_EnPWM = 0;
			motorflag_s.Theta_mode = savestate1;
			motorflag_s.Conrorl_mode = savestate2;
			theta_s.CountTeta = 0;
			Zerotime = 0;
			halfcountflag = 0;
			dir = 0;
		}
	}
}
