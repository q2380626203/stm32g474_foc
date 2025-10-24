/**
 ******************************************************************************
 * @file    user_main.h
 * @brief   用户主程序头文件 - STM32G474 FOC控制
 ******************************************************************************
 */

#ifndef USER_MAIN_H
#define USER_MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Exported functions --------------------------------------------------------*/
void User_Setup(void);  // 用户初始化函数
void User_Loop(void);   // 用户主循环函数

#ifdef __cplusplus
}
#endif

#endif /* USER_MAIN_H */

