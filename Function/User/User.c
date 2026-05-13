#include "Comm_usart.h"
#include "string.h"
#include "usart.h"
#include "motor.h"
#include "tim.h"
#include "motorlib.h"

void hardware_init(void);
void KeyPro(void);
void CommPro(void);



//key  led
__IO uint32_t  uwtick_key,uwtick_Comm;
unsigned char Key_Value,Key_Dowm,Key_Up,Key_Old;
int16_t Led1Value = 0;
//注入通道采样值
uint16_t   aADCxINJConvertedData[4];
//uint16_t   aADCxConvertedData[1];
uint16_t encoderCount;

void SysInit(void)
{
	hardware_init();
	
	parameter_init();
	
	//ShowMessage();
}

//主循环
/**********************************Mainloop******************************/


void MainLoop(void)
{

		//CANMeaagePro();
		
		KeyPro();//按键处理
	
	  CommPro();//串口通信处理
	


}
/******************************MainloopEND******************************/

/******************************中断回调函数******************************/
//ADC中断
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc)
{
	
	//ADC采样获取
	aADCxINJConvertedData[0] = hadc->Instance->JDR1;
	aADCxINJConvertedData[1] = hadc->Instance->JDR2;
	aADCxINJConvertedData[2] = hadc->Instance->JDR3;
	aADCxINJConvertedData[3] = hadc->Instance->JDR4;
	
	encoderCount = __HAL_TIM_GET_COUNTER(&htim3);
	
	motor_control(); //电机控制程序
	
//	 countbase++;
	


}
//编码器中断回调
void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
	if(htim->Instance == TIM3)
	{
		if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_3)
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
		HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, s);
	else if(c == 2)
		HAL_GPIO_WritePin(LED2_GPIO_Port, LED2_Pin, s);
}

//按键翻转
void Led_TogglePin(unsigned char c)
{	
	if(c == 1)//
		HAL_GPIO_TogglePin(LED1_GPIO_Port, LED1_Pin);
	else if(c == 2)
		HAL_GPIO_TogglePin(LED2_GPIO_Port, LED2_Pin);
}

//按键扫描
unsigned char Key_Scan(void)
{
	unsigned char c = 0;
	if(HAL_GPIO_ReadPin(KEY1_GPIO_Port,KEY1_Pin)==GPIO_PIN_SET)
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
		Led_disp(1,1);	
		key1FlagSet(1);	
	}
	else//停机
	{			
		Led_disp(1,0);
		key1FlagSet(0);
	}
	
	static int iled2;
	if(iled2++>10)
	{
		Led_TogglePin(2);
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
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,c1);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,c2);
	__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,c3);
}
/*****************  PWM使能引脚控制 **********************/
void PWM_Enable(void)
{
	//打开PWM
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);
	//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);  //使能为0
		
}

void PWM_DisEnable(void)
{	
	 //关闭PWM
	//HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_SET);  //失能为1
	HAL_GPIO_WritePin(GPIOG, GPIO_PIN_12, GPIO_PIN_RESET);
		
}
/*****************  end  **********************/
