#include "main.h"
#include "spi.h"
#include "DRV8353.H"

DRV_8353_info DRV_info;

// ***** SPI读写函数 *****
uint16_t SPI_ReadWrite_DRV8353(uint16_t txData)
{
	uint16_t rxData = 0;
	DRV8353_NSS_LOW();
	HAL_SPI_TransmitReceive(&hspi3, (uint8_t*)&txData, (uint8_t*)&rxData, 1, 1000);
	DRV8353_NSS_HIGH();
	HAL_Delay(1); // 添加小延时确保CS信号稳定
	return rxData;
}

// ***** DRV8353初始化配置函数 *****
void Set_DRV8353(void)
{
	// 1. 使能芯片
	DRV8353_ENABLE_ON();
	HAL_Delay(1000);  // 等待芯片上电稳定（参考FOC_X使用1000ms）

	// 2. 关闭PWM控制（配置期间）
	DRV8353_PWM_cotr_OFF();
	HAL_Delay(10);

	// 3. 配置寄存器（与STM32F103完全一致，优化发热）

	// 0x02: 驱动控制寄存器 (Driver Control Register)
	// 配置: 0x1420 - 3x PWM模式
	SPI_ReadWrite_DRV8353(0x1420);

	// 0x03: 高侧门极驱动寄存器 (Gate Drive HS)
	// 配置: 0x1955 - IDRIVEP_HS=50mA, IDRIVEN_HS=100mA (降低发热)
	SPI_ReadWrite_DRV8353(0x1955);

	// 0x04: 低侧门极驱动寄存器 (Gate Drive LS)
	// 配置: 0x2555 - IDRIVEP_LS=50mA, IDRIVEN_LS=100mA (降低发热)
	SPI_ReadWrite_DRV8353(0x2555);

	// 0x05: 过流保护控制寄存器 (OCP Control)
	// 配置: 0x296C - 死区时间800ns (与F103一致)
	SPI_ReadWrite_DRV8353(0x296C);

	// 0x06: CSA控制寄存器 (Current Sense Amplifier)
	// 配置: 0x32C3 - 电流采样放大器配置 (与F103一致)
	SPI_ReadWrite_DRV8353(0x32C3);

	// 4. 延时等待配置生效
	HAL_Delay(300);  // 使用FOC_X的300ms延时

	// 5. 读回所有寄存器验证配置
	DRV_info.TIMP0 = SPI_ReadWrite_DRV8353(0x8000); // 读地址0x00 - 故障状态1
	DRV_info.TIMP1 = SPI_ReadWrite_DRV8353(0x8800); // 读地址0x01 - 故障状态2
	DRV_info.TIMP2 = SPI_ReadWrite_DRV8353(0x9000); // 读地址0x02 - 驱动控制
	DRV_info.TIMP3 = SPI_ReadWrite_DRV8353(0x9800); // 读地址0x03 - 高侧驱动
	DRV_info.TIMP4 = SPI_ReadWrite_DRV8353(0xA000); // 读地址0x04 - 低侧驱动
	DRV_info.TIMP5 = SPI_ReadWrite_DRV8353(0xA800); // 读地址0x05 - OCP控制
	DRV_info.TIMP6 = SPI_ReadWrite_DRV8353(0xB000); // 读地址0x06 - CSA控制

	// 6. 开启PWM控制
	DRV8353_PWM_cotr_ON();
}

// ***** 故障检测函数 *****
uint8_t DRV8353_Check_Fault(void)
{
	uint16_t fsr1, fsr2;

	// 读取故障状态寄存器
	fsr1 = SPI_ReadWrite_DRV8353(0x8000); // FSR1
	fsr2 = SPI_ReadWrite_DRV8353(0x8800); // FSR2

	// 更新全局变量
	DRV_info.TIMP0 = fsr1;
	DRV_info.TIMP1 = fsr2;

	// 检查是否有故障
	if ((fsr1 & 0x07FF) != 0 || (fsr2 & 0x07FF) != 0) {
		return 1; // 有故障
	}

	return 0; // 无故障
}

// ***** 清除故障函数 *****
void DRV8353_Clear_Fault(void)
{
	// 向寄存器0x02的Bit 0 (CLR_FLT) 写1来清除故障
	// 需要读取当前配置并保持其他位不变
	uint16_t reg02 = SPI_ReadWrite_DRV8353(0x9000); // 读取当前值
	reg02 |= 0x0001; // 设置CLR_FLT位

	// 写回寄存器（地址0x02的写命令 = 0x1000 | 数据）
	SPI_ReadWrite_DRV8353(0x1000 | (reg02 & 0x07FF));

	HAL_Delay(10);

	// 重新读取故障状态
	DRV_info.TIMP0 = SPI_ReadWrite_DRV8353(0x8000);
	DRV_info.TIMP1 = SPI_ReadWrite_DRV8353(0x8800);
}
