
#include "as5047p_bsp_drv.h"
#include "math.h"

void AS5047P_bsp_init( AuroAS5047p *as5047p ){
	uint8_t i;
	_AS5047P_bsp_cs_H( as5047p );
	
	as5047p->angle = 0.0f;
	as5047p->last_angle = 0.0f;
	as5047p->speed = 0.0f;
	as5047p->postion = 0.0f;
	as5047p->angle_refresh_cnt = 0;
	as5047p->angle_refresh_cycle = 0.00005f; 
	as5047p->angle_refresh_limit = 5;
	as5047p->speed_param = ( 1.0f / ( as5047p->angle_refresh_cycle * as5047p->angle_refresh_limit )) *  0.017453292f; //
	as5047p->speed_rpm_param = 9.5238095f;
	as5047p->speed_sum = 0.0f;
	for( i = 0 ; i < 10 ; i++ )
		as5047p->speed_buf[i] = 0.0f;
//	_AS5047P_bsp_set_spi_mode(as5047p);
}
static uint16_t test = 0;
float AS5047P_bsp_read_angle( AuroAS5047p *as5047p , uint8_t daec_en ){
	uint16_t addr;
	uint16_t data;
	uint8_t i,num = 0;
	addr = daec_en ? 0xffff : 0x7ffe;
AS5047_RE_READ:
	while( 1 ){
		data = _AS5047P_bsp_recv( as5047p , addr );
		if( (data & 0x4000) == 0 ) 
			break;
	}
    for(i=0;i<15;i++){
        if((data >> i) & 1)
            num += 1;
    }
    if((((num & 0x01) == 1) && ((data >> 15) == 0)) || ( (num & 0x01) == 0) && ((data >> 15) == 1)){
		goto AS5047_RE_READ;
    }
	
	data = (data & 0x3fff);
	as5047p->angle =  (float)(data*360) / 16384.0f; 
		
	as5047p->angle_refresh_cnt += 1;
	if( as5047p->angle_refresh_cnt == as5047p->angle_refresh_limit ){
		float angle_error = ( as5047p->angle - as5047p->last_angle );
		
		if( angle_error > 240.0f )
			angle_error = angle_error - 360;
		else if( angle_error < (-240.0f) )
			angle_error = angle_error + 360;
		
		as5047p->speed = angle_error * as5047p->speed_param;
		as5047p->speed_sum -= as5047p->speed_buf[0];
		
		for( i = 0 ; i < 9 ; i++ ) {
			as5047p->speed_buf[i] = as5047p->speed_buf[ i + 1];
		}
		as5047p->speed_buf[9] = as5047p->speed;
		as5047p->speed_sum += as5047p->speed_buf[9];
		as5047p->speed = as5047p->speed_sum / 10.0f;
		as5047p->speed_rpm = as5047p->speed * as5047p->speed_rpm_param;
		as5047p->postion += angle_error;
		as5047p->last_angle = as5047p->angle;
		as5047p->angle_refresh_cnt = 0;
	}
	return as5047p->angle;
}

void _AS5047P_bsp_set_spi_mode( AuroAS5047p *as5047p ){
	if (HAL_SPI_DeInit( as5047p->hspi ) != HAL_OK)
    {
	//	usb_printf("111");
        while( 1 );
    }
    as5047p->hspi->Instance = as5047p->hspi->Instance;
    as5047p->hspi->Init.Mode = SPI_MODE_MASTER;
    as5047p->hspi->Init.Direction = SPI_DIRECTION_2LINES;
    as5047p->hspi->Init.DataSize = SPI_DATASIZE_16BIT;
    as5047p->hspi->Init.CLKPolarity = SPI_POLARITY_LOW;
    as5047p->hspi->Init.CLKPhase = SPI_PHASE_2EDGE;
    as5047p->hspi->Init.NSS = SPI_NSS_SOFT;
    as5047p->hspi->Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
    as5047p->hspi->Init.FirstBit = SPI_FIRSTBIT_MSB;
    as5047p->hspi->Init.TIMode = SPI_TIMODE_DISABLE;
    as5047p->hspi->Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
    as5047p->hspi->Init.CRCPolynomial = 10;
    if (HAL_SPI_Init( as5047p->hspi ) != HAL_OK)
    {
		//usb_printf("222");
        while( 1 );
    }
}


void _AS5047P_bsp_cs_H( AuroAS5047p *as5047p ){
	HAL_GPIO_WritePin(as5047p->CS_Port, as5047p->CS_Pin, GPIO_PIN_SET);
}

void _AS5047P_bsp_cs_L( AuroAS5047p *as5047p ){
	HAL_GPIO_WritePin(as5047p->CS_Port, as5047p->CS_Pin, GPIO_PIN_RESET);
}
void _AS5047P_bsp_send( AuroAS5047p *as5047p , uint16_t addr , uint16_t data ){
	volatile uint8_t delay = 20;
    uint8_t addrs[2];
    uint8_t datas[2];
    addrs[0] = addr >> 8;
    addrs[1] = addr & 0x00ff;
    datas[0] = data >> 8;
    datas[1] = data & 0x00ff;
    _AS5047P_bsp_cs_L( as5047p );
    HAL_SPI_Transmit( as5047p->hspi , addrs , 1 , 0xffff );
    _AS5047P_bsp_cs_H( as5047p );
	while( delay != 0 )
		delay--;
    _AS5047P_bsp_cs_L( as5047p );
    HAL_SPI_Transmit( as5047p->hspi , datas , 1 , 0xffff );
    _AS5047P_bsp_cs_H( as5047p );
}
uint16_t _AS5047P_bsp_recv( AuroAS5047p *as5047p , uint16_t addr ){
	volatile uint8_t delay = 20;
    uint16_t data;
    _AS5047P_bsp_cs_L( as5047p );
    HAL_SPI_TransmitReceive( as5047p->hspi , (uint8_t *)&addr , (uint8_t *)&data , 1 , 100 );
    _AS5047P_bsp_cs_H( as5047p );
   // addr = 0xC000;
    while( delay--)
    _AS5047P_bsp_cs_L( as5047p );
    HAL_SPI_TransmitReceive( as5047p->hspi , (uint8_t *)&addr , (uint8_t *)&data , 1 , 100 );
    _AS5047P_bsp_cs_H( as5047p );
    return data;
}
