/*
作者：拾小白电控
日期：2024.1.20
qq群：1群：860465413  
备注：代码仅供学习参考，未经允许，禁止商用或售卖


功能：通用功能函数
*/
#include "user.h"
#include "Comm_usart.h"
#include "string.h"
#include "usart.h"
#include "motor.h"
#include "tim.h"
#include "motorlib.h"
#include "drvSpi.h"

void hardware_init(void);
void KeyPro(void);
void CommPro(void);
void ZeroPro(void);
void ErrPro(void);

//key  led
__IO uint32_t  uwtick_key,uwtick_Comm,uwtick_Zero,uwtick_Err;  //定时器处理事件事件计数
unsigned char Key_Value,Key_Dowm,Key_Up,Key_Old;  //按键状态记录
int16_t Led1Value = 0;

uint16_t   aADCxINJConvertedData[4];  //注入采样数组
uint16_t   aADCxConvertedData[4];			//常规采样数组
uint16_t encoderCount;                //编码器计数值


void SysInit(void)
{
	hardware_init();  //硬件外设初始化
	
	parameter_init();	//电机参数初始化
	
}

//主循环
/**********************************Mainloop******************************/
void MainLoop(void)
{

		//CANMeaagePro();
		
		KeyPro();			//按键处理
	
	  CommPro();		//串口通信处理
	
	  ZeroPro();		//零点定位
	
		MonitorPro(); //状态监控
	
		ErrPro(); 		//故障处理
	

	
}

/******************************中断回调函数******************************/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)//ADC中断
{
	//ADC采样获取  三相电流和母线电压采集
	aADCxINJConvertedData[0] = hadc->Instance->JDR1;
	aADCxINJConvertedData[1] = hadc->Instance->JDR2;
	aADCxINJConvertedData[2] = hadc->Instance->JDR3;
	aADCxINJConvertedData[3] = hadc->Instance->JDR4;
	if(motorflag_s.Encorder == 1)
		encoderCount = __HAL_TIM_GET_COUNTER(&htim3);		//预留ABZ编码器
	else if(motorflag_s.Encorder == 2)
		encoderCount=AS5047_read(ANGLEUNC);  						//SPI读编码器，线数16384
	motor_control(); 																	//电机控制程序
}
//编码器中断回调
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)  //编码器Z相零点触发
		{
			__HAL_TIM_SET_COUNTER(&htim3,0);
		}
	}
}

/********************************GPIO******************************/   //ok
//灯
void Led_disp(unsigned char c,unsigned char state)
{
	GPIO_PinState s;
	state==1?(s = GPIO_PIN_RESET):(s = GPIO_PIN_SET);
	
	//GPIO置位
	if(c == 1)
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, s);   
//	else if(c == 2)
//		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, s);
}

//按键翻转
void Led_TogglePin(unsigned char c)
{	
	if(c == 1)//
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
//	else if(c == 2)
//		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
}

//按键扫描
unsigned char Key_Scan(void)
{
	unsigned char c = 0;
	if(HAL_GPIO_ReadPin(KEY_GPIO_Port,KEY_Pin)==GPIO_PIN_RESET)
		c = 1;
//	else if(HAL_GPIO_ReadPin(GPIOG,GPIO_PIN_2)==GPIO_PIN_SET)
//		c = 2;
	return c;
}
//按键和LED处理
void KeyPro(void)
{
	if(uwTick - uwtick_key<100) return;
	uwtick_key = uwTick;
  //实现每100ms执行一次函数

	Key_Value = Key_Scan();
	Key_Dowm = Key_Value&(Key_Value^Key_Old);
	Key_Up = ~Key_Value&(Key_Value^Key_Old);
  Key_Old = Key_Value;

	if(Key_Dowm == 1)
	{
		Led1Value = Led1Value^1;
	}
	else if(Key_Dowm == 2)
	{
		
	}		
	if(Led1Value == 1)//使能
	{
		key1FlagSet(1);	
	}
	else//停机
	{			
		key1FlagSet(0);
	}
	
	static int iled2;
	if(iled2++>1)
	{
		Led_TogglePin(1);
		iled2 = 0;
	}
}

/**************************GPIOEND******************************/
void CommPro(void)
{
	if(uwTick - uwtick_Comm<1) return;
	uwtick_Comm = uwTick;	

	//示波器通信
	ScopeCommPro();	
}
/**************************EPWM******************************/
void Set_SVPWM_Compare(int16_t c1,int16_t c2,int16_t c3)
{
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,c1);   //pwm1比较值
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,c2);		//pwm2比较值
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,c3);		//pwm3比较值
}
/*****************  PWM使能引脚控制 **********************/
void PWM_Enable(void)
{
	//打开PWM
	HAL_GPIO_WritePin(PWNEN_GPIO_Port, PWNEN_Pin, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);  //使能为0	
}

void PWM_DisEnable(void)
{	
	 //关闭PWM
	//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);  //失能为1
	HAL_GPIO_WritePin(PWNEN_GPIO_Port, PWNEN_Pin, GPIO_PIN_RESET);
}

void ZeroPro(void)
{
	if(uwTick - uwtick_Zero<10) return;
	uwtick_Zero = uwTick;
	ZeroFind();    //零点对齐
}

void ErrPro(void)
{
	if(uwTick - uwtick_Err<100) return;
	uwtick_Err = uwTick;
	
	if(HAL_GPIO_ReadPin(GPIOB,GPIO_PIN_11)==GPIO_PIN_RESET) //屏蔽硬件过流
	{
		Motor_Bit16.bits.HardErr = 1;
		__nop();
	}
	else
	{
		Motor_Bit16.bits.HardErr = 0;
		__nop();
	}
	
	if(motorflag_s.ERR_OVERCURRENT == 1) //软件过流
	{
		Motor_Bit16.bits.SoftErr = 1;
	}
	else
	{
		Motor_Bit16.bits.SoftErr = 0;
	}
	
	
	if(motorflag_s.ERR_UDC == 1)  //母线异常
	{
		Motor_Bit16.bits.UdcErr = 1;
	}
	else
	{
		Motor_Bit16.bits.UdcErr = 0;
	}
	
	if((Motor_Bit16.u16_Motorflag&(0xfe))!=0)  //故障自动恢复
	{
			motorflag_s.Key_EnPWM = 0;
			Led1Value = 0;
	}
		
	if(motorflag_s.ResetErr == 1)  //手动故障复位,硬件故障无法复位，需要重启
	{
		motorflag_s.ResetErr = 0;
		motorflag_s.ERR_OVERCURRENT = 0;
		motorflag_s.ERR_UDC = 0;
		Motor_Bit16.u16_Motorflag = 0;
	}	
}



/*****************  end  **********************/
