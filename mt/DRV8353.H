#ifndef drv8353_H
#define drv8353_H

#include <stdint.h>

// ***** DRV8353引脚定义 *****
// 硬件连接说明：
// - DRV_CS (PB0): SPI片选信号
// - PWM_COTR (PC15): 连接到 INLA/INLB/INLC (3-PWM模式的低侧控制)
// - ENABLE (PC14): 芯片使能信号
//
// PWM模式说明 (3-PWM模式):
// - INHA/INHB/INHC: 接STM32的TIM1_CH1/CH2/CH3 (高侧PWM)
// - INLA/INLB/INLC: 接STM32的PWM_COTR引脚 (低侧统一控制)
// - 3-PWM模式下，低侧开关由INLX信号控制，高侧由INHX的PWM调制

#define DRV8353_NSS_LOW()       HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_RESET)     // SPI CS拉低
#define DRV8353_NSS_HIGH()      HAL_GPIO_WritePin(DRV_CS_GPIO_Port, DRV_CS_Pin, GPIO_PIN_SET)       // SPI CS拉高
#define DRV8353_PWM_cotr_OFF()  HAL_GPIO_WritePin(PWM_COTR_GPIO_Port, PWM_COTR_Pin, GPIO_PIN_RESET) // 低侧关闭(INLX=0)
#define DRV8353_PWM_cotr_ON()   HAL_GPIO_WritePin(PWM_COTR_GPIO_Port, PWM_COTR_Pin, GPIO_PIN_SET)   // 低侧使能(INLX=1)
#define DRV8353_ENABLE_ON()     HAL_GPIO_WritePin(ENBALE_GPIO_Port, ENBALE_Pin, GPIO_PIN_SET)       // 使能芯片
#define DRV8353_ENABLE_OFF()    HAL_GPIO_WritePin(ENBALE_GPIO_Port, ENBALE_Pin, GPIO_PIN_RESET)     // 关闭芯片

// ***** DRV8353F 寄存器定义 *****
// 注意：DRV8353FSRTAR 型号没有内置电流采样放大器，只有6个寄存器(0x00-0x05)
typedef struct {
	uint16_t TIMP0;  // 0x00: 故障状态寄存器1 (只读) - VDS/OCP/UVLO/OTSD/GDF故障
	uint16_t TIMP1;  // 0x01: 故障状态寄存器2 (只读) - VGS/CPUV/OTW故障
	uint16_t TIMP2;  // 0x02: 驱动控制寄存器 - PWM模式/故障控制
	uint16_t TIMP3;  // 0x03: 高侧门极驱动寄存器 - 驱动电流配置
	uint16_t TIMP4;  // 0x04: 低侧门极驱动寄存器 - 驱动电流配置
	uint16_t TIMP5;  // 0x05: 过流保护控制寄存器 - OCP/死区时间配置
	uint16_t TIMP6;  // 0x06: 保留(DRV8353F无此寄存器，读回永远为0)
} DRV_8353_info;

// ***** 外部电流采样配置 *****
// 硬件: INA240A (增益=20V/V) + 0.5mΩ采样电阻
// 计算: Vout = I_phase × R_sense × Gain = I × 0.0005 × 20 = I × 0.01
// 例如: 10A → 0.1V, 50A → 0.5V, 100A → 1V

// ***** 全局变量声明 *****
extern DRV_8353_info DRV_info;  // DRV8353寄存器信息（定义在DRV8353.C中）

// ***** 函数声明 *****
void Set_DRV8353(void);
uint8_t DRV8353_Check_Fault(void);  // 返回0=无故障, 非0=有故障
void DRV8353_Clear_Fault(void);     // 清除故障标志

// ***** 3-PWM模式使用说明 *****
/*
 * DRV8353配置为3-PWM模式 (PWM_MODE = 01)
 *
 * 硬件连接:
 *   STM32              DRV8353F
 *   TIM1_CH1    -->    INHA (高侧A相PWM)
 *   TIM1_CH2    -->    INHB (高侧B相PWM)
 *   TIM1_CH3    -->    INHC (高侧C相PWM)
 *   PWM_COTR    -->    INLA/INLB/INLC (低侧开关控制，三路并联)
 *
 * 工作原理:
 *   - INHA/B/C: 输入高侧MOSFET的PWM信号（由定时器产生，0-100%占空比）
 *   - INLA/B/C: 输入低侧MOSFET的开关信号（GPIO控制，仅0或1）
 *   - PWM_COTR=1: 使能低侧MOSFET（续流路径）
 *   - PWM_COTR=0: 关闭低侧MOSFET（高阻态）
 *
 * FOC控制流程:
 *   1. 初始化: Set_DRV8353() → PWM_COTR自动设为1
 *   2. 运行前: 确保TIM1输出PWM（CCR1/2/3设置占空比）
 *   3. 换相时: 保持PWM_COTR=1，通过调整TIM1_CCRx实现SVPWM
 *   4. 停机时: PWM_COTR=0或ENABLE=0
 *
 * 注意事项:
 *   ⚠️ 启动前必须先调用 Set_DRV8353() 配置寄存器
 *   ⚠️ PWM频率建议20-40kHz（平衡效率与噪音）
 *   ⚠️ 死区时间由芯片硬件生成（默认50ns）
 *   ⚠️ 低侧INLX通常保持高电平，仅紧急情况拉低
 *
 * 故障处理:
 *   - 定期调用 DRV8353_Check_Fault() 检测故障
 *   - 发生故障时芯片自动关断输出
 *   - 排除故障原因后调用 DRV8353_Clear_Fault() 恢复
 */

#endif
