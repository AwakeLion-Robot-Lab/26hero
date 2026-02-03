#include "dma.h"

/*USART3*/
uint8_t  dbus_buf[2][DBUS_MAX_LEN];
/*USART6*/
uint8_t  image_buf[2][IMAGE_MAX_LEN];
/*USART2*/
uint8_t  usart2_TXbuf[USART2_BUFLEN];
uint8_t  usart2_RXbuf[USART2_BUFLEN];

/*USART1*/
uint8_t  pc_rxbuf[2][PC_MAX_LEN];
/*SPI5*/
uint8_t  spi1_rx_buf[SPI1_BUFLEN];
uint8_t  spi1_tx_buf[SPI1_BUFLEN];

void USART3_DMA(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA1, ENABLE);
	USART_DMACmd(USART3, USART_DMAReq_Rx, ENABLE);
	
	DMA_DeInit(DMA1_Stream1);
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART3->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)dbus_buf[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = DBUS_MAX_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA1_Stream1,&DMA_InitStructure);
	/*使用双缓冲模式*/
	DMA_DoubleBufferModeConfig(DMA1_Stream1,(uint32_t)dbus_buf[1],DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA1_Stream1,ENABLE);
	
	DMA_Cmd(DMA1_Stream1, DISABLE);
	DMA_Cmd(DMA1_Stream1, ENABLE);

}


void USART1_DMA(void)
{
	DMA_InitTypeDef DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	USART_DMACmd(USART1, USART_DMAReq_Rx, ENABLE);
	DMA_DeInit(DMA2_Stream5);
	
	DMA_InitStructure.DMA_Channel = DMA_Channel_4;
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)pc_rxbuf[0];
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize = PC_MAX_LEN;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream5,&DMA_InitStructure);
  
  /*接收使用双缓冲模式*/
	DMA_DoubleBufferModeConfig(DMA2_Stream5,(uint32_t)pc_rxbuf[1],DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA2_Stream5,ENABLE);

	DMA_Cmd(DMA2_Stream5, DISABLE);
	DMA_Cmd(DMA2_Stream5, ENABLE);
	
}


void USART6_DMA(void)//Image
{
	DMA_InitTypeDef USART6_DMA_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
	DMA_DeInit(DMA2_Stream1);
	
	USART6_DMA_InitStructure.DMA_Channel = DMA_Channel_5;
	USART6_DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(USART6->DR);
	USART6_DMA_InitStructure.DMA_Memory0BaseAddr = (uint32_t)image_buf[0];
	USART6_DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralToMemory;
	USART6_DMA_InitStructure.DMA_BufferSize = IMAGE_MAX_LEN;
	USART6_DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	USART6_DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	USART6_DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	USART6_DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	USART6_DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
	USART6_DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
	USART6_DMA_InitStructure.DMA_FIFOMode = DMA_FIFOMode_Disable;
	USART6_DMA_InitStructure.DMA_FIFOThreshold = DMA_FIFOThreshold_Full;
	USART6_DMA_InitStructure.DMA_MemoryBurst = DMA_MemoryBurst_Single;
	USART6_DMA_InitStructure.DMA_PeripheralBurst = DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream1,&USART6_DMA_InitStructure);
  
  /*接收使用双缓冲模式*/
	DMA_DoubleBufferModeConfig(DMA2_Stream1,(uint32_t)image_buf[1],DMA_Memory_0);
	DMA_DoubleBufferModeCmd(DMA2_Stream1,ENABLE);

	DMA_ITConfig(DMA2_Stream1,DMA_IT_TC,ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;  // 原为6，降低优先级
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);


	DMA_Cmd(DMA2_Stream1, DISABLE);
	DMA_Cmd(DMA2_Stream1, ENABLE);
	
}














