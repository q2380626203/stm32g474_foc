#ifndef __AUROPID_H_
#define __AUROPID_H_

#include "main.h"



typedef struct AuroPID{
	float P;
	float I;
	float D;
	
	float error;
	float last_error;
	float acc_integral;
	float I_integral_maxlimit;
	float I_integral_minlimit;
	float out_maxlimit;
	float out_minlimit;
}AuroPID;

float PID_Init( AuroPID *pid , float P , float I , float D , float integral_limit);
float PI_cal( AuroPID *pid , float actual_value , float desired_value);
float PID_cal( AuroPID *pid , float actual_value , float desired_value);

float Vec_PID_cal( AuroPID *pid , float actual_value , float gyro);
#endif
