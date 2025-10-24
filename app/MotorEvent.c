/**
 ******************************************************************************
 * @file    MotorEvent.c
 * @brief   电机事件管理 - STM32G474 + DRV8353 + AS5047P
 * @note    移植自Nebula_st_mdk项目,适配当前硬件
 ******************************************************************************
 */

#include "MotorEvent.h"
#include "AuroFOC.h"
#include "AuroPID.h"
#include "as5047p_bsp_drv.h"
#include "DRV8353.H"
#include "adc.h"
#include "tim.h"
#include "spi.h"

AuroFOC motorA;

/**
 * @brief  电机启动初始化
 * @note   配置FOC参数并启动PWM和ADC
 */
void MotorEvent_Start(void){
	// ===== 1. DRV8353驱动初始化 =====
	Set_DRV8353();  // 配置DRV8353寄存器并使能输出
	HAL_Delay(100);  // 等待DRV8353稳定

	// ===== 2. 硬件句柄绑定 =====
	motorA.motorID = 10;
	motorA.htim = &htim1;
	motorA.hadc = &hadc1;

	// ===== 3. AS5047P编码器初始化 (使用STM32G474的引脚) =====
	motorA.as5047p.hspi = &hspi1;
	motorA.as5047p.CS_Pin = SEN_CS_Pin;      // PB2 (当前项目引脚)
	motorA.as5047p.CS_Port = SEN_CS_GPIO_Port;
	AS5047P_bsp_init(&motorA.as5047p);

	// ===== 4. 电压和SVPWM参数 =====
	motorA.Uq = 0.8;    // 初始q轴电压(降低以减少发热)
	motorA.Ud = 0.0;    // d轴电压为0
	motorA.Ts = 2125;   // STM32G474 TIM1_ARR值(中心对齐模式)
	motorA.Udc = 12.0;  // 母线电压12V
	motorA.K = SQRT_3 * motorA.Ts / motorA.Udc;

	// ===== 5. 限制参数 =====
	motorA.limit.max_uq = 6;
	motorA.limit.max_ud = 6;
	motorA.limit.max_iq = 10;   // 最大电流10A (INA240A + 0.5mΩ)
	motorA.limit.max_id = 10;
	motorA.limit.max_speed = 500;  // 最大速度500 rad/s
	motorA.limit.max_position = 5000;
	motorA.limit.min_uq = -6;
	motorA.limit.min_ud = -6;
	motorA.limit.min_iq = -10;
	motorA.limit.min_id = -10;
	motorA.limit.min_speed = -500;
	motorA.limit.min_position = -5000;

	// ===== 6. 电机参数 (首次运行需校准) =====
	motorA.Pole_Pair = 7;              // 默认极对数(需校准)
	motorA.motor_rotate_direct = 1;    // 默认旋转方向(需校准)
	motorA.mechanical_offset = 0.0;    // 默认机械零偏(需校准)

	// ===== 7. 控制模式和目标值 =====
	motorA.focmode = stop_loop_mode;   // 初始停止模式
	motorA.aim.aim_iq = 0;
	motorA.aim.aim_id = 0;
	motorA.aim.aim_speed = 0;
	motorA.aim.aim_position = 0;

	// ===== 8. PID控制器初始化 =====
	PID_Init(&motorA.iq_pid, 2.5f, 0.0045f, 0, 300);
	PID_Init(&motorA.id_pid, 2.1f, 0.0012f, 0, 300);
	PID_Init(&motorA.speed_pid, 0.0985f, 0.00185f, 0, 400);
	PID_Init(&motorA.position_pid, 4.0f, 0.012f, 0, 200);

	// ===== 9. 启动ADC和PWM =====
	foc_start_adc(&motorA);   // 启动ADC注入转换
	foc_start_pwm(&motorA);   // 启动TIM1 PWM输出
}

/** adc注入中断**/
void HAL_ADCEx_InjectedConvCpltCallback(ADC_HandleTypeDef* hadc){
    if( hadc == &hadc1){
		foc_update_adc( &motorA , hadc1.Instance->JDR1 , hadc1.Instance->JDR2 , 0 );
		foc_control( &motorA );
    }
}
