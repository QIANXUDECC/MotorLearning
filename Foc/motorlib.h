#include "main.h"
#include "Table.h"
#include "MotorConfig.h"

#define abs(x) ((x < 0) ? -x : x)



//外部给定参数结构体
typedef struct __PARASET
{
	int32_t Postionloopref;  //累计旋机械角度（°）
	float speedloopref;
	float openloop_Idref;
	float openloop_Iqref;
	float openloop_Udref;
	float openloop_Uqref;	
	
}PARASET_s;

//MotolFlag
typedef struct __MOTORFLAG
{
	int16_t EnableCurrent; //电流环使能	
	int16_t EnableSpeed; //转速环使能标志	
	int16_t Conrorl_mode;	//电机发波控制方式  1电压给定 2电流给定 3速度环 4位置环 
	int16_t Theta_mode; //角度给定方式         0 角度自增  1编码器闭环 2固定位置
	
	int16_t Motor_EnableFlag;  //电机控制总使能,故障下关闭
	int16_t Key_EnPWM;					//手动按键使能开关
	int16_t ZeroFlag;  //找零点状态标志 0未开启  1进行中 2找零点完成
	int16_t Zero_EnPWM;//零点使能
	//故障标志位
	int16_t ERR_OVERCURRENT; //过流标志
	int16_t ERR_UDC; //过流标志
	int16_t ResetErr; //故障复位标志
	int16_t Encorder;  //编码器选择  0霍尔 1旋变 2绝对值
	
	int16_t version;
	
}MOTORFLAG_s;


//电流采样
typedef struct __ADCVALUE
{
		float Udc; //母线电压采样值
		int16_t Iu_Sample;   // U相电流采样值
		int16_t Iv_Sample;   // V相电流采样值
		int16_t Iw_Sample;   // W相电流采样值
		float Iu_Sample_offer;   // U相电流校正
		float Iv_Sample_offer;
		float Iw_Sample_offer;
		float Iu;    //u相采样电流
		float Iv;  //v相采样电流
		float Iw;  //w相采样电流

	
	
		float adc_refer; //参考0位电压
		float gain_cur;	//电流增益
		float gain_udc;  //母线增益
	
		int16_t adcstateflag;
		int16_t ADCcount;   //补偿计时器
	
		float currentoffera;
		float currentofferb;
		float currentofferc;
		
		int16_t currentofferEN;
	
}ADCVALUE_s;






//MotolFlag
typedef struct __THETA
{
	float ThetaSum; //速度控制周期内角度累加
	
		//编码器变量
	int16_t MotorPn;
	int16_t OnePulseNum; 
	int16_t PosInit; //初始位置
	int16_t encoder;    // 编码器单圈码盘脉冲
	int16_t encoderelc;    // 电角度脉冲
	int16_t realencoder;  //单圈机械位置
	int16_t Tetacnt;    // 电角度相位偏移量
	int16_t MotorTeta;  //电机机械角度 0-360
	int16_t Teta;    //电机电角度 0-360
	int16_t TetaRef;  //给定位置角度
	float Tetatmp;   //中间变量
	float TetaInc;  //自增电角度=给定频率
	float TetaIncInc;   //自增电角加速度
	int16_t OldTheta;    //上一个周期脉冲数记录
	int16_t NewTheta;    //本周期脉冲数记录
	int16_t ThetacountInc;    //周期内脉冲数增量	
	int16_t CountTeta;	
	
	int16_t Sin_Teta;    // 正弦值
	int16_t Cos_Teta;    // 余弦值
	
	uint16_t motordir;   //编码器旋转方向 0正常  1取反
	
	int16_t TetaSMO;    //无感算法辨识角度
	int16_t flag;      //无感切入闭环标志
	int16_t count;
	int16_t count2;
}THETA_s;



//SVPWM
typedef struct __EPWMSTRUCT
{
	int16_t Ualfa;
	int16_t Ubeta;
	int16_t epwm1;
	int16_t epwm2;
	int16_t epwm3;
  int32_t Udref;
	int32_t Uqref;
	
	int16_t A,B,C;
	int16_t cTime1,cTime2;
	int16_t Va,Vb,Vc;
	int16_t sector,t1,t2;
	int16_t	taon,tbon,tcon;
	//int16_t	epwm1,epwm2,epwm3;

	
}EPWM_s;

//电流环
typedef struct __CURRENTSTRUCT
{
	float Kp;  //电流环比例系数
	float Ki;  //电流环积分系数    (10000*ki)/s
	float Iqr; //q轴电流给定
	float Iq;  //q轴电流反馈
	float Idr; //d轴电流给定
	float Id;	 //d轴电流反馈
	float e_id;//d轴电流误差
	float x_id;//d轴电流积分
	float e_iq;//q轴电流误差
	float x_iq;//q轴电流积分
	float Udbuf;//d轴PI输出
	float Uqbuf;//q轴PI输出
	float Ud;   //d轴电压输出(限幅后)
	float Uq;		//q轴电压输出(限幅后)
	float Currentout_MAX;  //电流环输出最大值，即给定电压最大值
	
	
	float Ia; 
	float Ib; 
	float Ic; 
	float Ialfa;
	float Ibeta;
	

	
	
}CURRENT_s;

////滑动滤波器
//typedef struct __HUADONGLVBO
//{
//	uint16_t iCurrent;          //滑动滤波
//	float IdFbArr[16];
//	float IdFbSum;
//	float IqFbArr[16];
//	float IqFbSum;
//	
//}LVBO_s;

////位置环参数
//typedef struct __POSITION
//{
//	float Kp;
//	
//	
//}POSITION_s ,*position_s;


//速度环参数
typedef struct __SPEEDSTRUCT
{
	
	float SpeedTarget;  // 目标转速  r/min
	float SpeedRef;  //实际给定速度  r/min

	float x_speed;        // 速度偏差和
	float Kp;
	float Ki;
	float Obs_Kp;
	float Obs_Ki;
	float e_speed;  //速度偏差err
	float speed_UP;  //比例项
	float speed_UI;		//积分项
	float SPEED_OUT;  //速度PI控制器输出
	float Speed_OutPreSat;  //转速换输出值预处理
	float SpeedMAX;  //转速限幅	，转速最大值
	float SpeedoutMAX; //转速输出最大值，即电流给定最大值
	
	uint16_t SpeedLoopPrescaler;      // Speed loop pre scalar
	uint16_t SpeedLoopCount;           // Speed loop counter
	
	float speedminus;  //真实转速
	float speedshow;  //转速显示值
	float spdFbSum;
	int16_t ispd;
	float spdFbArr[10];

	
}SPEED_s;

//位置环参数
typedef struct __POSITIONSTRUCT
{
	int32_t PostionSum;  //累计旋电角度脉冲 = PostionSum*2000/360
	float Kp;
	float PostionOut;
	float PostionOutMAX;
	float Postion;    //绝对位置(°)
	
}
POSITION_s;

typedef struct __SCOPE
{
	int16_t scope1;
	int16_t scope2;
	int16_t scope3;
	int16_t scope4;

}SCOPE_s;

//换算参数
typedef struct __MONITOR
{
	int32_t Uref;  //给定电压
	int32_t Iact;  //实际电流
	int32_t Speedact; //实际转速    
	int32_t Motor_Bit16;   //电机状态位
	int32_t Conrorl_mode;  //控制模式
	int32_t Theta_mode;				//角度模式
	
}MONITOR_s;

typedef union {
    struct {
        unsigned int RUN:1;
        unsigned int HardErr:1;
        unsigned int SoftErr:1;
        unsigned int UdcErr:1;
        unsigned int bit4:1;
        unsigned int bit5:1;
        unsigned int bit6:1;
        unsigned int bit7:1;
				unsigned int bit8:1;
				unsigned int bit9:1;
				unsigned int bit10:1;
				unsigned int bit11:1;
				unsigned int bit12:1;
				unsigned int bit13:1;
			  unsigned int bit14:1;
				unsigned int bit15:1;
    } bits;
    unsigned int u16_Motorflag;
}MOTOR_BIT16;

extern MOTOR_BIT16 Motor_Bit16;
extern int16_t DATATABLE[30];
extern int16_t* data_Arr16[30];
extern int32_t* data_Arr32[30];
extern float *fdata_Arr[30];
extern int32_t* i32OnlyRead_Arr[10];
extern ADCVALUE_s adcvalue_s;	
extern MOTORFLAG_s motorflag_s;
extern CURRENT_s current_s;
extern THETA_s theta_s;
extern EPWM_s epwm_s; 
extern PARASET_s paraset_s;
extern SPEED_s speed_s;
extern SCOPE_s scope_s;
extern POSITION_s position_s;
extern MONITOR_s Monitor_s;


void runcur_process(CURRENT_s *current,EPWM_s *epwm);

void SVPWM(CURRENT_s *current,THETA_s *theta,uint16_t pwm_count,EPWM_s *epwm);

void Park_abctodq(CURRENT_s *current,THETA_s *theta);

//void dq_huadonglvbo(CURRENT_s *current,LVBO_s *lvbo);

void CurLoopInit(CURRENT_s *current);

void SpeedLoopInit(SPEED_s *speed);

void PositionLoop(POSITION_s *position,SPEED_s * speed,THETA_s *theta);

void SpeedLoop(SPEED_s * speed,CURRENT_s *current,THETA_s *theta);

void ThetaPro(THETA_s *theta,MOTORFLAG_s *mode);

void Adcpro(ADCVALUE_s *adcvalue,CURRENT_s *current,MOTORFLAG_s *motorflag ,uint16_t* Iraw);

void coderpro(THETA_s *theta,uint16_t encoderCount);

void key1FlagSet(uint16_t keyvalue);

float Limit(float value, float lim);

void datasave(void);

void MonitorPro(void);

