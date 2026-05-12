


#include "UART3.h"
#include "usart.h"




void DMA_RX_INIT(UART_HandleTypeDef *huartx,\
								 DMA_HandleTypeDef *hdma_usartx_rx,\
								 uint8_t *rx1_buf,\
								 uint8_t *rx2_buf,\
								 uint16_t dma_buf_num)
{
	SET_BIT(huartx->Instance->CR3, USART_CR3_DMAR);
	__HAL_UART_ENABLE_IT(huartx, UART_IT_IDLE);
	__HAL_DMA_DISABLE(huartx->hdmarx);
  while(hdma_usartx_rx->Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(huartx->hdmarx);
  }
	hdma_usartx_rx->Instance->PAR = (uint32_t) & (huartx->Instance->DR);
	hdma_usartx_rx->Instance->M0AR = (uint32_t)(rx1_buf);
	hdma_usartx_rx->Instance->M1AR = (uint32_t)(rx2_buf);
  hdma_usartx_rx->Instance->NDTR = dma_buf_num;
	SET_BIT(hdma_usartx_rx->Instance->CR, DMA_SxCR_DBM);
	__HAL_DMA_ENABLE(huartx->hdmarx);
}

//void DMA_TX_INIT(UART_HandleTypeDef *huartx)
//{
//	SET_BIT(huartx->Instance->CR3, USART_CR3_DMAT);
//}

//void DMA_TX_START(DMA_HandleTypeDef *hdma_usartx_tx,
//									uint8_t *data,
//									uint16_t len)
//{
//		__HAL_DMA_DISABLE(hdma_usartx_tx);
//    while(hdma_usartx_tx->Instance->CR & DMA_SxCR_EN)
//    {
//        __HAL_DMA_DISABLE(hdma_usartx_tx);
//    }

//    //clear flag
//    //清除标志位
//    __HAL_DMA_CLEAR_FLAG(hdma_usartx_tx, DMA_HISR_TCIF7);
//    __HAL_DMA_CLEAR_FLAG(hdma_usartx_tx, DMA_HISR_HTIF7);

//    //set data address
//    //设置数据地址
//    hdma_usartx_tx->Instance->M0AR = (uint32_t)(data);
//    //set data length
//    //设置数据长度
//    hdma_usartx_tx->Instance->NDTR = len;

//    //enable DMA
//    //使能DMA
//    __HAL_DMA_ENABLE(hdma_usartx_tx);
//}

//void usart_printf(DMA_HandleTypeDef *hdma_usartx_tx,const char *fmt,...)
//{
//    static uint8_t tx_buf[256] = {0};
//    static va_list ap;
//    static uint16_t len;
//    va_start(ap, fmt);

//    //return length of string 
//    //返回字符串长度
//    len = vsprintf((char *)tx_buf, fmt, ap);

//    va_end(ap);

//    DMA_TX_START(hdma_usartx_tx,tx_buf, len);

//}

/*-------------------------------------
* 函数名：串口DMA初始化
* 描述  ：初始化CV的串口及DMA接收
* 输入  ：串口，串口dma，缓冲0，缓冲1，数据长度
* 输出  ：无
作者：LPGUAIA魔改的DJI的
日期：2021.7.7
-----------------------------------------*/ 
Move_Date movedate={{0,0,0,0},{60,60,60,60},{2,2,2,2}};
uint8_t rx_buf[2][RX_BUF_NUM];
uint8_t Ut_ready=0;
void Ut_init(void)
{
//	DMA_TX_INIT(&huart3);
 	DMA_RX_INIT(&huart3,&hdma_usart3_rx,rx_buf[0],rx_buf[1],RX_BUF_NUM);
}

/*-------------------------------------
* 函数名：串口三解码
* 描述  ：解码串口三收到的内容
* 输入  ：串口三buffer，解码到的结构体
* 输出  ：无
作者：LPGUAIA魔改的DJI的
日期：2021.7.7
-----------------------------------------*/ 

void RX_Decoder(uint8_t *buf, Move_Date *Date)
{
	Ut_ready=0x01;
	
	if (buf == NULL)
  {
    return;
  }
	if (buf[0]!=0x00&&buf[0]!=0x01&&buf[0]!=0x02&&buf[0]!=0x03) //数据头不对，则重新开始寻找0x00数据头
	{
		return;
	}
	else
	{
		switch(buf[0])
		{
			case 0x00:
			{
				if(buf[11]==0x0A)
				{
					for (int i=0;i<4;i++)
						Date->taranglec[i]=( (buf[2*i+1] << 8)|buf[2*i+2]);
				}
			}break;
			case 0x01:
			{
				if(buf[11]==0x0A)
				{
					for (int i=0;i<4;i++)
						Date->tarspeed[i]=( (buf[2*i+1] << 8)|buf[2*i+2]);
				}
			}break;
			case 0x02:
			{
				if(buf[11]==0x0A)
				{
					for(int i=0;i<4;i++)
						Date->tartorque[i]=( (buf[2*i+1] << 8)|buf[2*i+2]);
				}
			}break;
			/*case 0x03:
			{
				if(buf[11]==0x0A)
				{
					for(int i=0;i<4;i++)
						clawinfo.state[i]=0x00;
				}
			}break;*/
//			case 0x04:
//			{
//				if(buf[11]==0x0A)
//				{
//					for(int i=0;i<4;i++)
//						clawinfo.state[i]=0x00;
//				}
//			}break;
		}
		
	}
}

/*-------------------------------------
* 函数名：放在it.c文件里的串口三的中断里的handler
* 描述  ：中断串口三接收处理
* 输入  ：无
* 输出  ：无
作者：LPGUAIA魔改的DJI的
日期：2021.7.7

作者：LPGUAIA抄的DJI的
日期：2021.12.22
-----------------------------------------*/ 



void USART3_inIRQHandler(void)
{
	if (huart3.Instance->SR & UART_FLAG_RXNE)
	{
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
	}
	else if (huart3.Instance->SR & UART_FLAG_IDLE)
	{
		static uint16_t this_time_rx_len = 0;
		__HAL_UART_CLEAR_PEFLAG(&huart3);

		if ((hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) == RESET)
		{
			__HAL_DMA_DISABLE(&hdma_usart3_rx);
			this_time_rx_len = RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
			hdma_usart3_rx.Instance->NDTR = RX_BUF_NUM;
			hdma_usart3_rx.Instance->CR |= DMA_SxCR_CT;
			__HAL_DMA_ENABLE(&hdma_usart3_rx);
			if (this_time_rx_len == FRAME_LENGTH)
			{
				RX_Decoder(rx_buf[0], &movedate);
			}
		}
		else
		{

			__HAL_DMA_DISABLE(&hdma_usart3_rx);
			this_time_rx_len = RX_BUF_NUM - hdma_usart3_rx.Instance->NDTR;
			hdma_usart3_rx.Instance->NDTR = RX_BUF_NUM;
			DMA1_Stream1->CR &= ~(DMA_SxCR_CT);
			__HAL_DMA_ENABLE(&hdma_usart3_rx);
			if (this_time_rx_len == FRAME_LENGTH)
			{
				RX_Decoder(rx_buf[1], &movedate);
			}
		}
	}
}
