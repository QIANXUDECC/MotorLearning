#include "main.h"

//extern uint16_t  encoderCount;
//extern uint16_t timer;
//extern uint16_t Motor_EnableFlag;
//extern uint16_t key_EnablePWM;



void motor_control(void);
void parameter_init(void);
void datasave(void);

void PWM_Enable(void);
void PWM_DisEnable(void);
void ZeroFind(void);//零点定位

