/*
作者：拾小白电控
日期：2024.4.20
qq群：1群：860465413
备注：代码仅供学习参考，未经允许，禁止商用或售卖

功能：基于反电动势的滑模观测器无传感器算法，该算法改编自Ti的电机库
*/

#include "Sensorless_SMO.h"
#include "motorlib.h"
#include "MotorConfig.h"

Angle_SMO Angle_SMOPare = Angle_SMO_DEFAULTS;
Speed_est Speed_estPare = Speed_est_DEFAULTS;
SMO_Motor SMO_MotorPare = SMO_Motor_DEFAULTS;
IQAtan IQAtan_Pare = IQAtan_DEFAULTS;

extern uint16_t VF_AngleJZ;
float Angle_CompCoff = 0.13; // 角度补偿系数，在线调节系数，观察输出能力
// 原来是0.15，vd、vq输出都为正，调节为0.11时vd能为负，但快速启动会失败。
int16_t Angle_Comp = 0;

float Limit_Sat(float Uint, float U_max, float U_min) // 限制赋值函数
{
	float Uout;
	if (Uint <= U_min)
		Uout = U_min;
	else if (Uint >= U_max)
		Uout = U_max;
	else
		Uout = Uint;
	return Uout;
}

void SMO_Pare_init(void) // 电机参数初始化
{
	SMO_MotorPare.Rs = 0.12f;		// 电机电阻欧姆
	SMO_MotorPare.Ls = 0.0003f;		// 电机电感H，电感不准也没关系，实际电感可能更小
	SMO_MotorPare.Ib = 10.0f;		// 电机的额定相电流，硬件可测得的最大相电流
	SMO_MotorPare.Vb = VOLTAGE_LIM; // 电机最大相电压
	SMO_MotorPare.Ts = 0.0001f;		// 代码运行周期，PWM中断周期 10KHZ
	SMO_MotorPare.POLES = Pn;		// 电机极对数
	SMO_MotorPare.Fsmopos = exp((-SMO_MotorPare.Rs / SMO_MotorPare.Ls) * (SMO_MotorPare.Ts));
	SMO_MotorPare.Gsmopos = (SMO_MotorPare.Vb / SMO_MotorPare.Ib) * (1.0f / SMO_MotorPare.Rs) * (1 - SMO_MotorPare.Fsmopos);

	// 计算滑膜 观测器系数， 根据实际电机调节滤波系数  Kslide和 Kslf
	Angle_SMOPare.Fsmopos = SMO_MotorPare.Fsmopos;
	Angle_SMOPare.Gsmopos = SMO_MotorPare.Gsmopos;
	Angle_SMOPare.Kslide = 0.17f;		   //
	Angle_SMOPare.Kslf = 0.06f;			   //
	Angle_SMOPare.E0 = VOLTAGE_LIM / 4.0f; //  限幅
	Speed_estPare.speed_coeff = (float)(500 * 60 / (SMO_MotorPare.POLES * 1024.0f));
}

void Angle_Cale(p_Angle_SMO pV)
{
	float IalphaError_Limit = 0, IbetaError_Limit = 0;

	pV->E0 = 4.0;
	// 滑模电流观测，求出Ialfa、Ibeta估计值
	pV->EstIalpha = pV->EstIalpha * pV->Fsmopos + (pV->Valpha - pV->Ealpha - pV->Zalpha) * pV->Gsmopos;
	pV->EstIbeta = pV->EstIbeta * pV->Fsmopos + (pV->Vbeta - pV->Ebeta - pV->Zbeta) * pV->Gsmopos;

	// 电流估计误差
	pV->IalphaError = pV->EstIalpha - pV->Ialpha;
	pV->IbetaError = pV->EstIbeta - pV->Ibeta;

	// 限幅函数
	IalphaError_Limit = Limit_Sat(pV->IalphaError, pV->E0, -pV->E0);
	IbetaError_Limit = Limit_Sat(pV->IbetaError, pV->E0, -pV->E0);

	// 滑模控制计算
	pV->Zalpha = IalphaError_Limit * pV->Kslide;
	pV->Zbeta = IbetaError_Limit * pV->Kslide;

	// 一阶低通滤波，求出反电势
	pV->Ealpha = pV->Ealpha + (pV->Zalpha - pV->Ealpha) * pV->Kslf;
	pV->Ebeta = pV->Ebeta + (pV->Zbeta - pV->Ebeta) * pV->Kslf;
}

void Angle_Correct(void)
{
	static int tmp;
	Angle_Comp = speed_s.SpeedRef * Angle_CompCoff;
	if (Angle_Comp > 100)
		Angle_Comp = 100;

	if (abs(tmp - IQAtan_Pare.IQAngle) > 150 && abs(tmp - IQAtan_Pare.IQAngle) < 850)
	{
		// 过滤掉干扰带来的异常角度
	}
	else
	{
		// 硬件和软件滤波导致的角度滞后补偿
		IQAtan_Pare.IQAngle_JZ = IQAtan_Pare.IQAngle + Angle_Comp;
	}
	tmp = IQAtan_Pare.IQAngle;

	if (IQAtan_Pare.IQAngle_JZ > 1024) // 调整电角度方位IQ10格式
		IQAtan_Pare.IQAngle_JZ = IQAtan_Pare.IQAngle_JZ - 1024;
	else if (IQAtan_Pare.IQAngle_JZ < 0)
		IQAtan_Pare.IQAngle_JZ = IQAtan_Pare.IQAngle_JZ + 1024;
	static int icount;
	if (icount++ > speed_s.SpeedLoopPrescaler)
	{
		SMO_Speedcale();
		icount = 0;
	}
}

float stest1 = 0.2f;
float stest2 = 0.8f;
void SMO_Speedcale(void) // 1ms执行一次
{
	Speed_estPare.SpeedK1 = stest1;
	Speed_estPare.SpeedK2 = stest2;
	Speed_estPare.ele_angle = (float)IQAtan_Pare.IQAngle_JZ;

	if (fabs(Speed_estPare.ele_angle - Speed_estPare.old_ele_angleIQ) < 500.0f)
	{
		Speed_estPare.Speed_ele_angle = Speed_estPare.ele_angle - Speed_estPare.old_ele_angleIQ;
	}

	Speed_estPare.Speed_ele_angleIQFitter = Speed_estPare.SpeedK2 * Speed_estPare.Speed_ele_angleIQFitter + Speed_estPare.SpeedK1 * Speed_estPare.Speed_ele_angle;
	Speed_estPare.Speed_RPM = Speed_estPare.Speed_ele_angleIQFitter * 1000 * 60 / 1024 / 7; // 最大角度 2pi是一圈 1024 = 360°

	if (Speed_estPare.Speed_RPM >= 5000)
		Speed_estPare.Speed_RPM = 5000;
	Speed_estPare.old_ele_angleIQ = Speed_estPare.ele_angle;

	// 低通滤波器，备用
	const float fc = 200;	 // 截止频率
	const float Ts = 0.0001; // 采样周期
	float b = 2 * 3.1415926f * fc * Ts;
	float alpha = b / (b + 1); // alpha

	Speed_estPare.Speed_RPM2 = Speed_estPare.Speed_RPM2 + alpha * (Speed_estPare.Speed_RPM - Speed_estPare.Speed_RPM2);
}

const int32_t IQAtan2_Table[256] = {
	0x00000000, 0x000000C9, 0x00000192, 0x0000025B, 0x00000324, 0x000003ED, 0x000004B6, 0x00000580,
	0x00000649, 0x00000712, 0x000008A7, 0x00000971, 0x00000A3B, 0x00000B05, 0x00000BD0, 0x00000C9B,
	0x00000D66, 0x00000E31, 0x00000EFD, 0x00000FC9, 0x00001095, 0x00001162, 0x00001218, 0x0000122F,
	0x000012FC, 0x000013CA, 0x00001498, 0x00001566, 0x00001635, 0x00001705, 0x000017D4, 0x000018A5,
	0x00001975, 0x00001A47, 0x00001B19, 0x00001BEB, 0x00001CBE, 0x00001D91, 0x00001E65, 0x00001F3A,
	0x0000200F, 0x000020E5, 0x000021BC, 0x00002294, 0x0000236C, 0x00002444, 0x0000251E, 0x000025F8,
	0x000026D4, 0x000027B0, 0x0000288C, 0x0000296A, 0x00002A49, 0x00002B28, 0x00002C08, 0x00002CEA,
	0x00002DCC, 0x00002EAF, 0x00002F94, 0x00003079, 0x00003160, 0x00003247, 0x00003330, 0x00003419,
	0x00003509, 0x00003571, 0x000036DE, 0x000037CD, 0x000038BD, 0x000039AE, 0x00003AA0, 0x00003B94,
	0x00003C8A, 0x00003D80, 0x00003E79, 0x00003F72, 0x0000406E, 0x0000416A, 0x00004269, 0x00004369,
	0x0000446A, 0x0000456E, 0x00004673, 0x0000477A, 0x00004882, 0x0000498D, 0x00004A99, 0x00004BA8,
	0x00004CB8, 0x00004DCA, 0x00004EDF, 0x00004FF5, 0x0000510E, 0x00005228, 0x00005345, 0x00005465,
	0x00005586, 0x000056AA, 0x000057D1, 0x000058FA, 0x00005A25, 0x00005B53, 0x00005C84, 0x00005DB8,
	0x00005EEE, 0x00006027, 0x00006163, 0x000062A2, 0x000063E4, 0x00006529, 0x00006671, 0x000067BD,
	0x0000690C, 0x00006A5E, 0x00006BB3, 0x00006D0D, 0x00006E69, 0x00006FCA, 0x0000712E, 0x00007296,
	0x00007403, 0x00007573, 0x000076E7, 0x00007860, 0x000079DD, 0x00007B5F, 0x00007CE5, 0x00007E70,
	0x00008000, 0x00008194, 0x0000832E, 0x000084CD, 0x00008671, 0x0000881A, 0x000089CA, 0x00008B7F,
	0x00008D39, 0x00008EFA, 0x000090C1, 0x0000928F, 0x00009463, 0x0000963D, 0x0000981F, 0x00009A08,
	0x00009BF7, 0x00009DEF, 0x00009FEE, 0x0000A1F5, 0x0000A404, 0x0000A61B, 0x0000A83B, 0x0000AA64,
	0x0000AC96, 0x0000AED1, 0x0000B116, 0x0000B365, 0x0000B5BE, 0x0000B822, 0x0000BA91, 0x0000BD0B,
	0x0000BF90, 0x0000C222, 0x0000C4C0, 0x0000C76A, 0x0000CA22, 0x0000CCE7, 0x0000CF88, 0x0000D29D,
	0x0000D58E, 0x0000D88E, 0x0000DB9F, 0x0000DEC0, 0x0000E1F3, 0x0000E538, 0x0000E88F, 0x0000EBFA,
	0x0000EF78, 0x0000F30B, 0x0000F6B4, 0x0000FA74, 0x0000FE4A, 0x00010239, 0x00010641, 0x00010A64,
	0x00010EA2, 0x000112FC, 0x00011774, 0x00011C0B, 0x00012C03, 0x0001259C, 0x00012A99, 0x00012FBC,
	0x00013504, 0x00013A76, 0x00014012, 0x000145DB, 0x00014BD3, 0x000151FD, 0x0001585A, 0x00015EEE,
	0x000165BC, 0x00016CC6, 0x00017411, 0x00017B9F, 0x00018376, 0x00018B98, 0x0001940A, 0x00019CD2,
	0x0001A5F5, 0x0001AF78, 0x0001B963, 0x0001C3BB, 0x0001CE88, 0x0001D9D2, 0x0001E5A3, 0x0001F205,
	0x0001FF01, 0x00020CA4, 0x00021AFB, 0x00022A15, 0x00023A02, 0x00024AD4, 0x00025C9F, 0x00026F7B,
	0x0002837F, 0x000298CA, 0x0002AF7C, 0x0002C7BA, 0x0002E1AE, 0x0002FD8A, 0x00031B84, 0x00033BDF,
	0x00035EE7, 0x000384F5, 0x0003AE73, 0x0003DBDD, 0x00040DCB, 0x000444F4, 0x00048236, 0x0004C6A6,
	0x0005139B, 0x00056AC9, 0x0005CE63, 0x00064144, 0x0006C740, 0x0007658D, 0x00082374, 0x00090B81,
	0x000A2D7F, 0x000BA246, 0x000D9338, 0x00104AD7, 0x00145E24, 0x001B28CC, 0x0028BDDA, 0x00517C7E};

#define _IQdiv(A, B) (int32_t)((A << 15) / B)
#define Abs(A) ((A >= 0) ? A : -A)
void IQAtan_Cale(p_IQAtan pV)
{
	int16_t i = 0;
	if (pV->Alpha == 0)
	{
		if (pV->Beta == 0)
			i = 0;
		else
			i = 256;
	}
	else
	{
		pV->IQTan = _IQdiv(Abs(pV->Alpha), Abs(pV->Beta));
		if (IQAtan2_Table[i + 128] <= pV->IQTan)
			i += 128;
		if (IQAtan2_Table[i + 64] <= pV->IQTan)
			i += 64;
		if (IQAtan2_Table[i + 32] <= pV->IQTan)
			i += 32;
		if (IQAtan2_Table[i + 16] <= pV->IQTan)
			i += 16;
		if (IQAtan2_Table[i + 8] <= pV->IQTan)
			i += 8;
		if (IQAtan2_Table[i + 4] <= pV->IQTan)
			i += 4;
		if (IQAtan2_Table[i + 2] <= pV->IQTan)
			i += 2;
		if (IQAtan2_Table[i + 1] <= pV->IQTan)
			i += 1;
	}
	if (pV->Alpha == 0)
	{
		pV->IQAngle = i + 256;
	}
	else if (pV->Alpha > 0)
	{
		if (pV->Beta == 0)
			pV->IQAngle = 256;
		else if (pV->Beta > 0)
			pV->IQAngle = i;
		else
			pV->IQAngle = 512 - i;
	}
	else
	{
		if (pV->Beta == 0)
			pV->IQAngle = i + 768;
		else if (pV->Beta > 0)
			pV->IQAngle = 1024 - i;
		else
			pV->IQAngle = i + 512;
	}
	if (pV->IQAngle < 0)
		pV->IQAngle += 1024;
	// pV->IQAngle= pV->IQAngle<<6;
	Angle_Correct();
}
