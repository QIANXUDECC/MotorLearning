#include "main.h"
#include "tim.h"

void MainLoop(void);//主循环初始化

void SysInit(void);//上电初始化

void Set_SVPWM_Compare(int16_t c1,int16_t c2,int16_t c3);

void PWM_Enable(void);

void PWM_DisEnable(void);
