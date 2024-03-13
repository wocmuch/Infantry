#ifndef _OPS_ACTION_H
#define _OPS_ACTION_H

#include "main.h"
#include "usart.h"
typedef  struct
{
	float pos_x;//×ø±êX--ZBx
	float pos_y;//×ø±êY--ZBy
	float zangle;//º½Ïò½Ç
	float xangle;//¸©Ñö½Ç
	float yangle;//ºá¹ö½Ç
	float w_z;//º½Ïò½ÇËÙ

	float set_pos_x;
	float set_pos_y;
	float set_zangle;
	int move_flag;
	
} ops_t;

void opsusart_receive_handler(UART_HandleTypeDef *huart,DMA_HandleTypeDef*hdma_usart_rx);
ops_t* get_location_control_point(void);
#endif
