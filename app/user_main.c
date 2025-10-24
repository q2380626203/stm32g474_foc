/**
  ******************************************************************************
  * @file           : user_main.c
  * @brief          : User main program - FOC核心控制
  * @note           : 移植自Nebula_st_mdk项目,适配STM32G474+DRV8353
  ******************************************************************************
  * 硬件配置:
  *   - MCU: STM32G474RBT6 @ 170MHz
  *   - 驱动: DRV8353FSRTAR (3-PWM模式)
  *   - 编码器: AS5047P (SPI1)
  *   - 电流采样: INA240A + 0.5mΩ (ADC1注入通道)
  *   - PWM频率: 20kHz (TIM1中心对齐模式)
  *   - FOC控制频率: 20kHz (ADC注入中断)
  ******************************************************************************
  */

#include "user_main.h"
#include "MotorEvent.h"
#include "DRV8353.H"

/* Private variables ---------------------------------------------------------*/
static uint32_t loop_counter = 0;

/**
  * @brief  用户初始化函数
  * @note   在main()的MX_xxx_Init()之后调用
  */
void User_Setup(void)
{
    HAL_Delay(500);  // 等待系统稳定

    // 启动电机事件管理 (包含DRV8353初始化、FOC初始化、ADC/PWM启动)
    MotorEvent_Start();

    HAL_Delay(200);  // 等待ADC零偏校准完成
}

/**
  * @brief  用户主循环函数
  * @note   在main()的while(1)中调用
  *         FOC控制在ADC中断中执行(20kHz)
  *         主循环可执行低优先级任务
  */
void User_Loop(void)
{
    loop_counter++;

    // 示例: 每1秒执行一次低优先级任务
    static uint32_t last_time = 0;
    if (HAL_GetTick() - last_time >= 1000) {
        last_time = HAL_GetTick();

        // 在这里可以添加:
        // - 参数调整
        // - 状态监控
        // - 通信处理
        // - 故障检测等
    }

    HAL_Delay(10);  // 避免主循环占用过多CPU
}
