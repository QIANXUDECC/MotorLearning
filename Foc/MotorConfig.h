/*
作者：拾小白电控
日期：2024.1.20
qq群：1群：860465413 
备注：代码仅供学习参考，未经允许，禁止商用或售卖


功能：FOC参数定义
*/
#include "stdint.h"

#define DATA0 30

#define DATACOUNT DATA0

#define PWMCOUNT PWM_PERIOD    //pwm计数周期 根据定时器设定值得到

#define VERSION 106


//电机参数
#define Pn 7   								//极对数
#define PluseNum 16384 				//2000 //1024  //编码器单圈脉冲数
#define DCBUSVALUE 12.0f  		//母线电压值
#define ENCODER 2         		//编码器选择  0霍尔 1旋变 2绝对值
#define CTR_MODE  1 					//控制模式   1电压给定 2电流给定 3速度环 4位置环 5无感控制
#define TETA_MODE  0 					//角度给定  0 角度自增  1编码器闭环 2固定位置 


//有传感器下的参数
#define SPEED_Kp 0.05f  				//转速环Kp
#define SPEED_Ki 0.00002f 				//转速环ki
#define CURRENT_Kp 0.017f   	//电流环Kp    0.00002(H)*300*2*pi = 0.037    ->3000r/min
#define CURRENT_Ki 0.002826f  	//电流环ki    0.15(Ω)*300*2*pi/10000 = 0.02826  
#define POSITION_Kp 0.02f   	//位置环Kp

//无传感器控制下速度环参数
#define Obs_SPEED_Kp 0.0005f  				//转速环Kp
#define Obs_SPEED_Ki 0.000002f 				//转速环ki
#define Obs_CURRENT_LIM 1.5f    	//最大给定电流

//限幅参数
#define PWM_PERIOD  8400    	//PWM计数周期值,即控制电压输出最大值以该值为准
#define VOLTAGE_LIM 6.0f   	//最大输出电压7V/12V   14/24V
#define CURRENT_LIM 2.0f    	//最大给定电流
#define CURRENTOVER_MAX 4.0f	//软件过流电流
#define SPEED_LIM 3000.0f   	//最大转速
#define POSITIONOUTMAX 2000.0f 

//电流采集相关
#define ADCREFER  	1.65  //1.24f		//adc电流0参考值偏移量
#define ADCCURGAIN  100.0f/16.5f//  // 50.0f/8.0f //uvw电流增益
#define ADCUDCGAIN 25.0f          	//udc母线增益
#define CURRENTOFFEREN 1     				//软件校正 1使用 0不使用

//示波器
#define SCOPEGROUP1 1								//示波器1默认
#define SCOPEGROUP2 10							//示波器2默认
#define SCOPEGROUP3 11							//示波器3默认
#define SCOPEGROUP4 12							//示波器4默认






