/*
作者：拾小白电控
日期：2024.1.20
qq群：1群：860465413  
备注：代码仅供学习参考，未经允许，禁止商用或售卖


功能：硬件参数初始化
*/
#include "main.h"
#include "adc.h"
//#include "can.h"
#include "dma.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "Comm_usart.h"

extern uint8_t receive_buff[BUFFER_SIZE];         
extern uint16_t  aADCxConvertedData[4];
void hardware_init(void)
{


		//外设底层配置
		//PWM_Enable();
		HAL_TIM_Base_Start(&htim1);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_1);
		HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_1);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_2);
		HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_2);
		HAL_TIM_PWM_Start(&htim1,TIM_CHANNEL_3);
		HAL_TIMEx_PWMN_Start(&htim1,TIM_CHANNEL_3);

		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_1,8400/2);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_2,8400/2);
		__HAL_TIM_SET_COMPARE(&htim1,TIM_CHANNEL_3,8400/2);


		//tim3 编码器
		HAL_TIM_Encoder_Start(&htim3,TIM_CHANNEL_ALL);
		__HAL_TIM_SetCounter(&htim3,0);
		HAL_TIM_IC_Start_IT(&htim3,TIM_CHANNEL_3);

		//CAN
		//CAN_Config();
 
		//串口
		__HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE); 
		//HAL_UARTEx_ReceiveToIdle_DMA(&huart1, (uint8_t*)receive_buff, BUFFER_SIZE);//设置DMA传输，将串口1的数据搬运到	    //f407用
		HAL_UART_Receive_DMA(&huart1, (uint8_t*)receive_buff, BUFFER_SIZE);
					
		//adc
		HAL_ADC_Start_DMA(&hadc1,(uint32_t *)aADCxConvertedData,4); //轮询注入二选一
		__HAL_ADC_ENABLE_IT(&hadc1,ADC_IT_JEOC);//  
		HAL_ADCEx_InjectedStart(&hadc1);
	 
}
