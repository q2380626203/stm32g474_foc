#ifndef __AUROFOC_H_
#define __AUROFOC_H_

#include "main.h"
#include "AuroPID.h"
#include "as5047p_bsp_drv.h"

#define SQRT_3 1.732050807568877f
#define SQRT_3DIV2 0.8660254037844386f


typedef struct AuroFOCAim{
	float aim_iq;
	float aim_id;
	float aim_speed;
	float aim_speeed_rpm;
	float aim_position;
}AuroAim;

typedef enum AuroFOCMode{
	stop_loop_mode = 0,
	selftest_loop_mode,
	calibration_angle_pole_pair_mode,
	open_loop_mode,
	i_loop_mode,
	speed_loop_mode,
	position_loop_mode
}AuroFOCMode;

typedef struct AuroFOCLimit{
	float max_uq;
	float max_ud;
	float min_uq;
	float min_ud;
	
	float max_iq;
	float max_id;
	float min_iq;
	float min_id;
	
	float max_speed;
	float min_speed;
	
	float max_position;
	float min_position;
}AuroFOCLimit;

typedef struct AuroFOC{
	uint16_t motorID;
	uint8_t Pole_Pair;		      //电机极对数
	uint8_t motor_rotate_direct;
	float mechanical_offset;      //电机零偏
	float e_angle;
	float Uq;
	float Ud;
	float Ualpha;
	float Ubeta;
	
	float Ia;
	float Ib;
	float Ic;
	float Ialpha;
	float Ibeta;
	float Iq;
	float Id;
	
	float K;
	float Ts;
	float Udc;
	float sinVal;
	float cosVal;
	
	AuroFOCMode focmode;
	AuroAim aim;
	AuroFOCLimit limit;
	AuroPID	iq_pid;
	AuroPID id_pid;
	AuroPID speed_pid;
	AuroPID position_pid;
	AuroAS5047p as5047p;
	
	
	TIM_HandleTypeDef *htim;
    ADC_HandleTypeDef *hadc;
}AuroFOC;







void foc_init( AuroFOC *foc );
void foc_halt_pwm( AuroFOC *foc );
void foc_start_pwm( AuroFOC *foc );
void foc_start_adc( AuroFOC *foc );
void foc_update_adc( AuroFOC *foc , uint32_t uu , uint32_t uv , uint32_t uw);

void foc_control( AuroFOC *foc );
void _foc_stop_loop( AuroFOC *foc );
void _foc_selftest_loop( AuroFOC *foc );
void _foc_open_loop( AuroFOC *foc );
void _foc_selftest_loop( AuroFOC *foc );
void _foc_open_loop( AuroFOC *foc );
void _foc_i_loop( AuroFOC *foc );
void _foc_speed_loop( AuroFOC *foc );
void _foc_position_loop( AuroFOC *foc ); 
void _foc_calibration_angle_pole_pair_mode( AuroFOC *foc );
void _foc_cogging_torque_compensate( AuroFOC *foc , float e_angle , uint8_t flag);
void _foc_get_angle( AuroFOC *foc );
void _foc_cal_sincos( AuroFOC *foc );
void _foc_clark( AuroFOC *foc );
void _foc_park( AuroFOC *foc );
void _foc_ipark( AuroFOC *foc );
void _foc_svpwm( AuroFOC *foc );
void _foc_update_pwm( AuroFOC *foc , float tu , float tv , float tw );

float _foc_value_limit( float value , float max , float min );

void _foc_delay(uint32_t time);



#endif
