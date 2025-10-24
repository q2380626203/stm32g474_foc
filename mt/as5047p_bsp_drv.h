#ifndef __AS5047P_BSP_DRV_H_
#define __AS5047P_BSP_DRV_H_


#include "main.h"
#include "spi.h"
#include "tim.h"
#include "gpio.h"


typedef struct AuroAS5047p{
	SPI_HandleTypeDef *hspi;
	GPIO_TypeDef* CS_Port;
    uint16_t CS_Pin;
	uint8_t  angle_refresh_cnt;
	uint8_t  angle_refresh_limit;
	float angle_refresh_cycle;
	float speed_rpm_param;
	float speed_param;
	float angle;
	float last_angle;
	float speed;
	float speed_rpm;
	float postion;
	
	float speed_buf[10];
	float speed_sum;
}AuroAS5047p;


void AS5047P_bsp_init( AuroAS5047p *as5047p );
float AS5047P_bsp_read_angle( AuroAS5047p *as5047p , uint8_t daec_en );



void _AS5047P_bsp_set_spi_mode( AuroAS5047p *as5047p );
void _AS5047P_bsp_cs_H( AuroAS5047p *as5047p );
void _AS5047P_bsp_cs_L( AuroAS5047p *as5047p );
void _AS5047P_bsp_send( AuroAS5047p *as5047p , uint16_t addr , uint16_t data );
uint16_t _AS5047P_bsp_recv( AuroAS5047p *as5047p , uint16_t addr );


#endif
