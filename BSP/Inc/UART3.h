#ifndef UART3_H
#define UART3_H

#include "usart.h"

#define RX_BUF_NUM 24

#define FRAME_LENGTH 12

void USART3_inIRQHandler(void);
	
typedef struct _Move_Date
{
	float taranglec[4];
	float tarspeed[4];
	float tartorque[4];
}Move_Date;

extern Move_Date movedate;


void DMA_RX_INIT(UART_HandleTypeDef *huartx,\
								 DMA_HandleTypeDef *hdma_usartx_rx,\
								 uint8_t *rx1_buf,\
								 uint8_t *rx2_buf,\
								 uint16_t dma_buf_num);
void Ut_init(void);
extern uint8_t Ut_ready;
#endif
