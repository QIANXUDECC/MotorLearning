
//############################################################
// FILE:  Sensorless_SMO.h
// Created on: 2017年1月18日
// Author: wyb
// summary: Sensorless_SMO
//############################################################  

#ifndef  Sensorless_SMO_H
#define  Sensorless_SMO_H
#include "main.h"
#include "math.h"

typedef struct 	{ 
	        int32_t  Alpha; 			// Input:   alpha-axis
				  int32_t  Beta;			// Input:   beta-axis
				  int32_t  IQTan;				// Output:  phase-a	
  				int32_t  IQAngle;		
	        int32_t  IQAngle_JZ;
         } IQAtan , *p_IQAtan;


#define IQAtan_DEFAULTS  {0,0,0,0,0}

		 typedef struct
		 {
			 float Valpha;		// 二相静止坐标系alpha-轴电压
			 float Ealpha;		// 二相静止坐标系alpha-轴反电动势
			 float Zalpha;		// alfa轴滑膜观测器的z平面
			 float EstIalpha;	// 滑膜估算alpha-轴电流
			 float Vbeta;		// 二相静止坐标系beta-轴电压
			 float Ebeta;		// 二相静止坐标系beta-轴反电动势
			 float Zbeta;		// beta轴滑膜观测器的z平面
			 float EstIbeta;	// 滑膜估算beta-轴电流
			 float Ialpha;		// 二相静止坐标系alpha-轴电流
			 float IalphaError; // 二相静止坐标系beta-轴电流误差
			 float Kslide;		// 滤波器系数
			 float Ibeta;		// 二相静止坐标系beta-轴电流
			 float IbetaError;	// 二相静止坐标系beta-轴电流误差
			 float Kslf;		// 滤波器系数
			 float E0;			// 滑膜的电流误差的限幅值 0.5
			 float Fsmopos;		// 滑膜系数1
			 float Gsmopos;		// 滑膜系数2
		 } Angle_SMO, *p_Angle_SMO;

#define Angle_SMO_DEFAULTS {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0} // 初始化参数

typedef struct {
        float   ele_angle;  //速度电角度值（计算速度）  
        float   old_ele_angleIQ;   // 电机历史电角度
        float   Speed_ele_angle;      // 电机电角度
        float   Speed_ele_angleIQFitter;  //速度电角度值（计算速度） 
        float   MXZ_State;	
        float   Speed_RPM;       	 //电机旋转速度 	 
        float   Speed_RPM2;       	 //电机旋转速度 	 
        float     speed_coeff;       //计算速度的系数
			  float SpeedK1;
				float SpeedK2;
	   }Speed_est;

#define Speed_est_DEFAULTS {0,0,0,0,0,0,0} // 初始化参数

typedef struct{
        float  Rs; 			//电机的相电阻	 
        float  Ls;			//电机的相电感	  
        float  Ib; 			//电机控制器的基本相电流 	  
        float  Vb;			//电机控制器的基本相电压	 
        float  Ts;			 //采样周期	 
        uint8_t  POLES; // 电机的极对数
        float  Fsmopos;		   //滑膜常数1
        float  Gsmopos;			 //滑膜常数2
  }SMO_Motor;

#define SMO_Motor_DEFAULTS {0,0,0,0,0,4,0,0} // 初始化参数

extern  Angle_SMO   Angle_SMOPare ;
extern  Speed_est   Speed_estPare ;
extern  SMO_Motor   SMO_MotorPare ;
extern  IQAtan      IQAtan_Pare; 

void  Angle_Correct(void);
void  Angle_Cale(p_Angle_SMO  pV); //滑膜电机位置电角度计算
void  SMO_Pare_init (void );       //滑膜观测器的参数初始化
void  SMO_Speedcale(void);             //滑膜的角度计算速度函数
void  IQAtan_Cale(IQAtan *pV);
#endif /* Sensorless_SMO*/










