
#include "AuroPID.h"






float PID_Init( AuroPID *pid , float P , float I , float D , float integral_limit){
	pid->P = P;
	pid->I = I;
	pid->D = D;
	pid->I_integral_maxlimit = integral_limit;
	pid->I_integral_minlimit = -integral_limit;
	pid->acc_integral = 0;
	pid->last_error = 0;
	pid->error = 0;

}



float PI_cal( AuroPID *pid , float actual_value , float desired_value){
	float value;
	float P_value;
	float I_value;
	pid->last_error = pid->error;
	pid->error = desired_value - actual_value;
	pid->acc_integral += pid->error;
	if( pid->acc_integral > pid->I_integral_maxlimit )
		pid->acc_integral = pid->I_integral_maxlimit;
	else if( pid->acc_integral < pid->I_integral_minlimit )
		pid->acc_integral = pid->I_integral_minlimit;		
	
	//arm_mult_f32( &pid->P , &pid->error , &P_value , 1);
	//arm_mult_f32( &pid->I , &pid->acc_integral , &I_value , 1);
	//arm_mult_f32( &P_value , &I_value , &value , 1);
	value = pid->P * pid->error + pid->I * pid->acc_integral;
	return value;
}

float PID_cal( AuroPID *pid , float actual_value , float desired_value){
	float value;
	float P_value;
	float I_value;
	float D_value;
	float D;
	pid->last_error = pid->error;
	pid->error = desired_value - actual_value;
	pid->acc_integral += pid->error;
	if( pid->acc_integral > pid->I_integral_maxlimit )
		pid->acc_integral = pid->I_integral_maxlimit;
	else if( pid->acc_integral < pid->I_integral_minlimit )
		pid->acc_integral = pid->I_integral_minlimit;	
	D = pid->error - pid->last_error;
	
	P_value = pid->P * pid->error;
	I_value = pid->I * pid->acc_integral;
	D_value = pid->D * D;
	value = P_value + I_value + D_value;
	return value;
}


float Vec_PID_cal( AuroPID *pid , float actual_value , float gyro){
	float value;
	float P_value;
	float D_value;
	float D;
	pid->error = 0 - actual_value;
	P_value = pid->P * pid->error;
	D_value = gyro * pid->D;
	value = P_value + D_value;
	return value;
}
