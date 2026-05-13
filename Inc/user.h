#include "main.h"
#include "tim.h"
#include "adc.h"
//#include "can.h"
#include "motor.h"
#include <stdio.h>



#define LED1_Pin GPIO_PIN_11
#define LED1_GPIO_Port GPIOB
#define KEY1_Pin GPIO_PIN_4
#define KEY1_GPIO_Port GPIOA


//void Usart_SendString(uint8_t *str);
extern UART_HandleTypeDef huart1;

void Led_disp(unsigned char c,unsigned char state);
unsigned char Key_Scan(void);
void Led_TogglePin(unsigned char c);

extern void PWM_Enable(void);
extern void PWM_DisEnable(void);
extern void Set_SVPWM_Compare(int16_t c1,int16_t c2,int16_t c3);

void MainLoop(void);
void SysInit(void);

//void USER_UART_IRQHandler(UART_HandleTypeDef *huart);




