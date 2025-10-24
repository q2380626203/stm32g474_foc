
#include "AuroFOC.h"
#include "math.h"


static uint32_t u_time = 0;
static uint32_t uu_sum = 0;
static uint32_t uv_sum = 0;
static uint32_t uu_offset = 0;
static uint32_t uv_offset = 0;

static uint8_t  compensate_flag = DISABLE;
float compensate_talbe[1800] = {0};

/* FOC初始化 */
void foc_init( AuroFOC *foc ){
	
}

/* 开始ADC采样 */
void foc_start_adc( AuroFOC *foc ){
	__HAL_ADC_ENABLE_IT( foc->hadc , ADC_IT_JEOC );
    HAL_ADCEx_InjectedStart( foc->hadc );
    // STM32G474不需要启动TIM_CHANNEL_4，ADC由TIM1_TRGO触发
}

/* 更新采样到的三相电流值 */
void foc_update_adc( AuroFOC *foc , uint32_t uu , uint32_t uv , uint32_t uw){
	//零偏校准
	if( u_time < 500 ){
		uu_sum += uu;
		uv_sum += uv;
		u_time += 1;
	}
	else if( u_time == 500){
		uu_offset = uu_sum / 500.0;
		uv_offset = uv_sum / 500.0;
		u_time += 1;
	}
	else;

    // STM32G474 ADC1: 12位分辨率, 参考电压3.3V, INA240A增益20V/V, 采样电阻0.5mΩ
    // 电流转换公式: I = (ADC - offset) * (3.3V / 4096) / (0.5mΩ * 20) = (ADC - offset) * 0.0003296875
    // 为提高精度，使用原始公式的优化版本
    float Uu_ = ((float)((int)uu - (int)uu_offset)) * 0.0008056640625f;  // 3.3 / 4096
    float Uv_ = ((float)((int)uv - (int)uv_offset)) * 0.0008056640625f;

    // 电流放大: Vout = I * R_sense * Gain = I * 0.0005 * 20 = I * 0.01
    // 因此: I = Vout / 0.01 = Vout * 100
    foc->Ia = foc->Ia * 0.2f + 0.8f * (Uu_ * 100.0f);  // 低通滤波 + 电流转换
	foc->Ib = foc->Ib * 0.2f + 0.8f * (Uv_ * 100.0f);
	foc->Ic = (-(foc->Ia + foc->Ib));
}

void foc_halt_pwm( AuroFOC *foc ){
	__HAL_TIM_SetCompare( foc->htim , TIM_CHANNEL_1 , 0 );//ARR_10Khz
    __HAL_TIM_SetCompare( foc->htim , TIM_CHANNEL_2 , 0 );//ARR_10Khz
    __HAL_TIM_SetCompare( foc->htim , TIM_CHANNEL_3 , 0 );//ARR_10Khz
}
void foc_start_pwm( AuroFOC *foc ){
	// DRV8353使用3-PWM模式，只需要3个高侧PWM通道
    HAL_TIM_PWM_Start( foc->htim , TIM_CHANNEL_1 );
    HAL_TIM_PWM_Start( foc->htim , TIM_CHANNEL_2 );
    HAL_TIM_PWM_Start( foc->htim , TIM_CHANNEL_3 );

    // DRV8353不需要互补PWM输出，低侧由PWM_COTR统一控制
    // HAL_TIMEx_PWMN_Start已删除

    // ===== 关键修复：手动使能TIM1主输出 (MOE位) =====
    // TIM1是高级定时器，必须设置MOE=1才能输出PWM
    // 原6-PWM代码中HAL_TIMEx_PWMN_Start()会自动设置MOE，但现在需要手动设置
    __HAL_TIM_MOE_ENABLE(foc->htim);

    __HAL_TIM_SET_COMPARE( foc->htim , TIM_CHANNEL_1 , 0 );
    __HAL_TIM_SET_COMPARE( foc->htim , TIM_CHANNEL_2 , 0 );
    __HAL_TIM_SET_COMPARE( foc->htim , TIM_CHANNEL_3 , 0 );

    __HAL_TIM_ENABLE_IT( foc->htim  ,TIM_IT_UPDATE ); //开启更新中断
}

//电机控制
void foc_control( AuroFOC *foc ){
	switch( foc->focmode ){
		case stop_loop_mode: _foc_stop_loop( foc ); break;
		case selftest_loop_mode: _foc_selftest_loop( foc ) ; break;
		case open_loop_mode: _foc_open_loop( foc ); break;
		case i_loop_mode: _foc_i_loop( foc ); break;
		case speed_loop_mode: _foc_speed_loop( foc ); break;
		case position_loop_mode: _foc_position_loop( foc ); break;
		case calibration_angle_pole_pair_mode: _foc_calibration_angle_pole_pair_mode( foc ); break;
		default: _foc_stop_loop( foc ); break;
	}
}
//暂停
void _foc_stop_loop( AuroFOC *foc ){
	//_foc_get_angle( foc );
	foc_halt_pwm( foc );
}

//自测
void _foc_selftest_loop( AuroFOC *foc ){
	foc->as5047p.angle += 0.04f;  //角度自增
	_foc_cal_sincos( foc );
    _foc_clark( foc );
    _foc_park( foc );
    _foc_ipark( foc ); 
    _foc_svpwm( foc );
}

//开环
uint32_t open_loop_cycle_cnt = 0;
static float open_loop_shaft_angle = 0.0f;      // 开环机械轴角度(度) [0-360]
static float open_loop_velocity = 600.0f;       // 开环目标速度(度/秒), 100RPM

void _foc_open_loop( AuroFOC *foc ){
	// ===== 开环模式：使用编码器角度 + Uq/Ud强制给定 =====
	// 这种模式适合：
	// 1. 验证编码器是否正常工作
	// 2. 测试FOC算法（Clarke/Park/SVPWM）
	// 3. 验证电机参数（极对数、机械偏差）
	// 4. 调试Uq/Ud响应特性
	//
	// 工作原理：
	// - 从编码器读取实际角度（闭环角度反馈）
	// - 直接给定Uq/Ud电压（开环电压控制）
	// - 电机会产生固定转矩，转速由负载决定

	// 1. 读取编码器角度（包含极对数转换和机械偏差补偿）
	_foc_get_angle( foc );

	// 2. 计算电角度的sin/cos
	_foc_cal_sincos( foc );

	// 3. Clarke变换（三相电流 -> 两相静止坐标系）
    _foc_clark( foc );

    // 4. Park变换（静止坐标系 -> 旋转坐标系，得到Id/Iq）
    _foc_park( foc );

	// ===== 5. 强制给定Uq/Ud（开环电压控制的核心）=====
	// Uq/Ud已经在MotorEvent.c初始化时设置：
	// - motorA.Uq = 0.8V（产生转矩）
	// - motorA.Ud = 0.0V（不需要励磁）
	// 运行中可以通过修改 foc->Uq 来调整转矩大小
	//
	// 注意：这里不需要写代码，直接使用结构体中的Uq/Ud值

	// 6. 逆Park变换（dq -> αβ）
    _foc_ipark( foc );

    // 7. SVPWM调制（αβ -> 三相PWM占空比）
    _foc_svpwm( foc );

	// ===== 8. 齿槽转矩补偿（可选功能）=====
	if( compensate_flag == DISABLE )
		_foc_cogging_torque_compensate( foc , foc->e_angle , 0 );
	if( open_loop_cycle_cnt == 50000 ){
		_foc_cogging_torque_compensate( foc , foc->e_angle , 1 );
		open_loop_cycle_cnt += 1;
		compensate_flag = ENABLE;
	}
	else if( open_loop_cycle_cnt == 50001 ){
		open_loop_cycle_cnt = open_loop_cycle_cnt;
	}
	else{
		open_loop_cycle_cnt += 1;
	}


}
//电流环
void _foc_i_loop( AuroFOC *foc ){
	_foc_get_angle( foc );
	_foc_cal_sincos( foc );
	_foc_clark( foc );
    _foc_park( foc );
	
	
	foc->Uq = PI_cal( &foc->iq_pid , foc->Iq , foc->aim.aim_iq );
	foc->Ud = PI_cal( &foc->id_pid , foc->Id , foc->aim.aim_id );
	
	//UqUd限制
	foc->Uq = _foc_value_limit( foc->Uq , foc->limit.max_uq , foc->limit.min_uq );
	foc->Ud = _foc_value_limit( foc->Ud , foc->limit.max_ud , foc->limit.min_ud );
	
	_foc_ipark( foc );
	_foc_svpwm( foc );
}
//速度环
void _foc_speed_loop( AuroFOC *foc ){
	_foc_get_angle( foc );
	_foc_cal_sincos( foc );
	_foc_clark( foc );
    _foc_park( foc );
		
	if( foc->as5047p.angle_refresh_cnt == 0 ){
		if( foc->motor_rotate_direct == ENABLE ){ //ENABLE
			foc->speed_pid.P = fabsf(foc->speed_pid.P);
			foc->speed_pid.I = fabsf(foc->speed_pid.I);
		}
		else{
			foc->speed_pid.P = -fabsf(foc->speed_pid.P);
			foc->speed_pid.I = -fabsf(foc->speed_pid.I);
		}
		
		foc->aim.aim_iq = PI_cal( &foc->speed_pid , foc->as5047p.speed , foc->aim.aim_speed );
		foc->aim.aim_iq = _foc_value_limit( foc->aim.aim_iq , foc->limit.max_iq , foc->limit.min_iq );
	}
	
	foc->Uq = PI_cal( &foc->iq_pid , foc->Iq , foc->aim.aim_iq );
	foc->Ud = PI_cal( &foc->id_pid , foc->Id , foc->aim.aim_id );
	
	foc->Uq = _foc_value_limit( foc->Uq , foc->limit.max_uq , foc->limit.min_uq );
	foc->Ud = _foc_value_limit( foc->Ud , foc->limit.max_ud , foc->limit.min_ud );
	
	_foc_ipark( foc );
	_foc_svpwm( foc );
}
//位置环
void _foc_position_loop( AuroFOC *foc ){
	_foc_get_angle( foc );
	_foc_cal_sincos( foc );
	_foc_clark( foc );
    _foc_park( foc );
		
	if( foc->as5047p.angle_refresh_cnt == 0 ){
		foc->aim.aim_speed = 0.01f * PI_cal( &foc->position_pid , foc->as5047p.postion , foc->aim.aim_position );
		
		foc->aim.aim_speed = _foc_value_limit( foc->aim.aim_speed , foc->limit.max_speed , foc->limit.min_speed );
		
		if( foc->motor_rotate_direct == ENABLE ){
			foc->speed_pid.P = fabsf(foc->speed_pid.P);
			foc->speed_pid.I = fabsf(foc->speed_pid.I);
		}
		else{
			foc->speed_pid.P = -fabsf(foc->speed_pid.P);
			foc->speed_pid.I = -fabsf(foc->speed_pid.I);
		}
		
		foc->aim.aim_iq = PI_cal( &foc->speed_pid , foc->as5047p.speed , foc->aim.aim_speed );
		foc->aim.aim_iq = _foc_value_limit( foc->aim.aim_iq , foc->limit.max_iq , foc->limit.min_iq );
	}
	
	foc->Uq = PI_cal( &foc->iq_pid , foc->Iq , foc->aim.aim_iq );
	foc->Ud = PI_cal( &foc->id_pid , foc->Id , foc->aim.aim_id );
	
	foc->Uq = _foc_value_limit( foc->Uq , foc->limit.max_uq , foc->limit.min_uq );
	foc->Ud = _foc_value_limit( foc->Ud , foc->limit.max_ud , foc->limit.min_ud );
	
	_foc_ipark( foc );
	_foc_svpwm( foc );
}
//电机校准
static uint32_t calibration_cnt = 0;
static int32_t	calibration_direction_cnt = 0;
static float calibration_angle = 0;
static float calibration_angle_reg = 0;
static float calibration_start;
static float calibration_end;
static uint16_t calibration_Pole_Pair;
void _foc_calibration_angle_pole_pair_mode( AuroFOC *foc ){
	float angle_temp;
	foc->Uq = 0.0f;
	foc->Ud = 1.0f;
	foc->Pole_Pair = 1;
	foc->as5047p.angle = calibration_angle;
	_foc_cal_sincos( foc );
    _foc_clark( foc );
    _foc_park( foc );
    _foc_ipark( foc ); 
    _foc_svpwm( foc );
	if( calibration_cnt < 10 ){ //获取起始角度
		calibration_angle += 0.0f;
		calibration_start = AS5047P_bsp_read_angle( &foc->as5047p , 1);
	}
	else if( calibration_cnt < 54010){    //正转 360*6 个电角度
		angle_temp = AS5047P_bsp_read_angle( &foc->as5047p , 1);
		calibration_angle += 0.04f;
		if( calibration_angle >= 360.0f)
			calibration_angle = 0;
		if( calibration_angle_reg < angle_temp && calibration_cnt % 40 == 0){
			calibration_direction_cnt += 1;
			calibration_angle_reg = angle_temp;
		}
		else if( calibration_cnt % 40 == 0 ){
			calibration_direction_cnt -= 1;
			calibration_angle_reg = angle_temp;
		}
		calibration_end = angle_temp;
		if( calibration_cnt == 54009 ){
			if( fabsf( calibration_end - calibration_start )  < 200 ) 
				calibration_Pole_Pair = ( 360 * 6) / fabsf( calibration_end - calibration_start ) ;
			else
				calibration_Pole_Pair = ( 360 * 6) / ( 360 - fabsf( calibration_end - calibration_start ) ) ;
		}
	}
	else if( calibration_cnt < 108010 ){  //反转 360*6 个电角度 
		angle_temp = AS5047P_bsp_read_angle( &foc->as5047p , 1);
		calibration_angle -= 0.04f;
		if( calibration_angle == 0 )
			calibration_angle = 360.0f;
		if( calibration_angle_reg > angle_temp && calibration_cnt % 40 == 0){
			calibration_direction_cnt += 1;
			calibration_angle_reg = angle_temp;
		}
		else if( calibration_cnt % 40 == 0 ){
			calibration_direction_cnt -= 1;
			calibration_angle_reg = angle_temp;
		}
		calibration_end = angle_temp;
	}
	calibration_cnt += 1;
	//校准完成
	if( calibration_cnt == 108010 ){
		foc->mechanical_offset = calibration_start;
		foc->Pole_Pair = calibration_Pole_Pair;
		if( calibration_direction_cnt > 0)
			foc->motor_rotate_direct = ENABLE;
		else
			foc->motor_rotate_direct = DISABLE;
		calibration_cnt = 0;
		calibration_direction_cnt = 0;
		calibration_start = 0;
		calibration_end = 0;
		foc->Uq = 0.7;
		foc->Ud = 0;
		foc->focmode = stop_loop_mode;
	}
}



/* 齿槽转矩补偿 */
void _foc_cogging_torque_compensate( AuroFOC *foc , float e_angle , uint8_t flag){
	if( flag == 0 ){
		uint16_t angle = (uint16_t)(e_angle * 5);
		compensate_talbe[angle] = fabsf(foc->as5047p.speed_rpm) * fabsf(foc->as5047p.speed_rpm) * fabsf(foc->as5047p.speed_rpm); //记录当前角度下的速度值
	}
	else{
		uint16_t i;
		float temp_angle[200];
		float mean_data = 0;
		float filiter_data = 0;
		//窗口滤波
		for(i = 0; i < 1799 ; i++ ){
			compensate_talbe[i] = ( compensate_talbe[i] + compensate_talbe[i+1]) / 2.0f;
		}
		for( i = 0 ; i < 1800 ; i++ )
			mean_data += compensate_talbe[i];
		mean_data = mean_data / 1800.0f;
		for( i = 0 ; i < 1800 ; i++ ){
			compensate_talbe[i] = fabsf(compensate_talbe[i] - mean_data) * 0.000002;
			compensate_talbe[i] = compensate_talbe[i] * compensate_talbe[i];
		}
		for( i= 0 ; i < 200 ; i++ )
			temp_angle[i] = compensate_talbe[i];
		for( i = 0 ; i < 1800 - 200 ; i++ )
			compensate_talbe[i] = compensate_talbe[i + 200 ];
		for( i = 1800 - 200 ; i < 1800 ; i++ )
			compensate_talbe[i] = temp_angle[i - 1800 + 200];
	}
}
void _foc_get_angle( AuroFOC *foc ){
	float angle;
	angle = AS5047P_bsp_read_angle( &foc->as5047p,1);

	if( angle < foc->mechanical_offset )
		angle = foc->mechanical_offset - angle;
	else
		angle = foc->mechanical_offset + ( 360.0f - angle );
	if( foc->motor_rotate_direct == ENABLE )
		foc->as5047p.angle = 360 - angle;
	else 
		foc->as5047p.angle = angle;
}
void _foc_cal_sincos( AuroFOC *foc ){
	float e_angle; //电角度
	e_angle = foc->as5047p.angle * ((float)foc->Pole_Pair);  // 强制类型转换
	while( e_angle > 360.f )
		e_angle -= 360.f;
	foc->e_angle = e_angle;
	e_angle *= 0.0174532922222f;
	
	
//	foc->sinVal = arm_sin_f32( e_angle );
//	foc->cosVal = arm_cos_f32( e_angle );
	
	foc->sinVal = sin( e_angle );
	foc->cosVal = cos( e_angle );
	
}
void _foc_clark( AuroFOC *foc ){
	foc->Ialpha = foc->Ia;
	foc->Ibeta  = 0.5773502691896f * ( foc->Ia +  2*foc->Ib );
	//arm_clarke_f32( foc->Ia , foc->Ib , &foc->Ialpha , &foc->Ibeta);
}
void _foc_park( AuroFOC *foc ){
	foc->Iq =  ( -foc->Ialpha * foc->sinVal + foc->Ibeta * foc->cosVal );   
	foc->Id =  ( foc->Ialpha * foc->cosVal + foc->Ibeta * foc->sinVal );  
	//arm_park_f32( foc->Ialpha , foc->Ibeta , &foc->Id , &foc->Iq , foc->sinVal , foc->cosVal);
}

void _foc_ipark( AuroFOC *foc ){
	float Uq_cogging = 0;
	float Uq;
	if( compensate_flag == ENABLE){
		Uq_cogging = compensate_talbe[(uint16_t)(foc->e_angle * 5)];
	}
	if( foc->Uq > 0 )
		Uq =  foc->Uq;//+ Uq_cogging;
	else
		Uq =  foc->Uq;// - Uq_cogging;
	
	
    foc->Ualpha = foc->Ud * foc->cosVal - Uq * foc->sinVal;
    foc->Ubeta  = Uq * foc->cosVal + foc->Ud * foc->sinVal;
	//arm_inv_park_f32( foc->Ud , foc->Uq , &foc->Ualpha , &foc->Ubeta , foc->sinVal , foc->cosVal);
}



void _foc_svpwm( AuroFOC *foc ){
 // float K = SQRT_3*Ts/Udc;
    float U1,U2,U3;
    float T0=0,T1,T2,T3,T4,T5,T6,T7;
    float Ta,Tb,Tc;
    float sqrt_u,Ubeta_div2;
    uint8_t A,B,C,N,sector;
    sqrt_u = SQRT_3DIV2 * foc->Ualpha;
    Ubeta_div2 =  foc->Ubeta/2;
    U1 = foc->Ubeta;
//    U2 = -SQRT_3DIV2 * Ualpha - Ubeta/2;
//    U3 = SQRT_3DIV2 * Ualpha - Ubeta/2;
    U2 = -sqrt_u - Ubeta_div2;
    U3 = sqrt_u - Ubeta_div2;
    A = (U1 > 0) ? 1 : 0;
    B = (U2 > 0) ? 1 : 0;
    C = (U3 > 0) ? 1 : 0;
    N = (A<<2) + (B<<1) + (C);

    //扇区选择
    sector = ( N == 5) ? 1:
             ( N == 4) ? 2:
             ( N == 6) ? 3:
             ( N == 2) ? 4:
             ( N == 3) ? 5: 6;
	//根据扇区选择矢量作用时间
    switch (sector) {
        case 1:
            T4 = foc->K * U3;
            T6 = foc->K * U1;
           //归一化
            if( foc->Ts < T4 + T6){
                float k = foc->Ts / (T4 + T6);
                T4 = k * T4;
                T6 = k * T6;
            }
            T7 = (foc->Ts - T4 -T6) / 2;
            Ta = T4 + T6 + T0 + T7;
            Tb = T6 + T0 + T7;
            Tc = T0 + T7;
            break;
        case 2:
            T2 = -foc->K * U3;
            T6 = -foc->K * U2;
            //归一化
            if( foc->Ts < T2 + T6){
                float k = foc->Ts / (T2 + T6);
                T2 = k * T2;
                T6 = k * T6;
            }
            T7 = (foc->Ts - T2 -T6) / 2;
            Ta = T6 + T0 + T7;
            Tb = T2 + T6 + T0 + T7;
            Tc = T0 + T7;
            break;
        case 3:
            T2 = foc->K*U1;
            T3 = foc->K*U2;
            //归一化
            if( foc->Ts < T2 + T3){
                float k = foc->Ts / (T2 + T3);
                T2 = k * T2;
                T3 = k * T3;
            }
            T7 = (foc->Ts - T2 -T3) / 2;

            Ta = T0 + T7;
            Tb = T2 + T3 + T0 + T7;
            Tc = T3 + T0 + T7;
            break;
        case 4:
            T1 = -foc->K*U1;
            T3 = -foc->K*U3;
            //归一化
            if( foc->Ts < T1 + T3){
                float k = foc->Ts / (T1 + T3);
                T1 = k * T1;
                T3 = k * T3;
            }

            T7 = (foc->Ts - T1 -T3) / 2;
            Ta = T0 + T7;
            Tb = T3 + T0 + T7;
            Tc = T1 + T3 + T0 + T7;
            break;
        case 5:
            T1 = foc->K*U2;
            T5 = foc->K*U3;
            //归一化
            if( foc->Ts < T1 + T5){
                float k = foc->Ts / (T1 + T5);
                T1 = k * T1;
                T5 = k * T5;
            }
            T7 = (foc->Ts - T1 -T5) / 2;

            Ta = T5 + T0 + T7;
            Tb = T0 + T7;
            Tc = T1 + T5 + T0 + T7;
            break;
        case 6:
            T4 = -foc->K*U2;
            T5 = -foc->K*U1;
            //归一化
            if( foc->Ts < T4 + T5){
                float k = foc->Ts / (T4 + T5);
                T4 = k * T4;
                T5 = k * T5;
            }
            T7 = (foc->Ts - T4 -T5) / 2;
            Ta = T4 + T5 + T0 + T7;
            Tb = T0 + T7;
            Tc = T5 + T0 + T7;
            break;
    };
    _foc_update_pwm( foc , Ta , Tb , Tc );
}
void _foc_update_pwm( AuroFOC *foc , float ta , float tb , float tc ){
	// ===== 关键修复：SVPWM输出的Ta/Tb/Tc是时间值，需要转换为占空比 =====
	// 参考F103项目 foc.c:461-468
	// 占空比 = 时间 / 周期
	float tu = ta / foc->Ts;
	float tv = tb / foc->Ts;
	float tw = tc / foc->Ts;

	// 限制占空比到 [0, 0.8]
	tu = (tu > 0.8f) ? 0.8f : (tu < 0.0f) ? 0.0f : tu;
    tv = (tv > 0.8f) ? 0.8f : (tv < 0.0f) ? 0.0f : tv;
    tw = (tw > 0.8f) ? 0.8f : (tw < 0.0f) ? 0.0f : tw;

	// STM32G474 TIM1配置: ARR=2125, 中心对齐模式, 20kHz PWM
	// CCR范围: 0-2125 (0%到100%占空比)
	foc->htim->Instance->CCR1 = (uint16_t)(tu * 2125);
    foc->htim->Instance->CCR2 = (uint16_t)(tv * 2125);
    foc->htim->Instance->CCR3 = (uint16_t)(tw * 2125);
}

float _foc_value_limit( float value , float max , float min ){
	if( value > max )
		return max;
	else if( value < min )
		return min;
	else
		return value;
}
void _foc_delay(uint32_t time){
	uint32_t i;
	while(time--){
		i = time;
		while(i--);
	}
}