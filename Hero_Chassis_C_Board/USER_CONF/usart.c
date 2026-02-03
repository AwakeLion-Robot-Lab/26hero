#include "usart.h"
#include "dma.h"

USART_InitTypeDef USART_InitStructure3;
void USART3_DEVICE(void)/*DBUS*/
{
	/**USART3 GPIO Configuration
	PC11     ------> USART3_RX
	PC10     ------> USART3_TX
	*/
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,ENABLE);
	
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB1PeriphResetCmd(RCC_APB1Periph_USART3, DISABLE);
	
	GPIO_PinAFConfig(GPIOC, GPIO_PinSource11, GPIO_AF_USART3);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOC,&GPIO_InitStructure);
	
	USART_DeInit(USART3);
	USART_InitStructure3.USART_BaudRate = 100000;
	USART_InitStructure3.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure3.USART_StopBits = USART_StopBits_1;
	USART_InitStructure3.USART_Parity = USART_Parity_Even;
	USART_InitStructure3.USART_Mode = USART_Mode_Rx;
	USART_InitStructure3.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART3,&USART_InitStructure3);
	
	USART_Cmd(USART3, ENABLE);
	USART_ClearFlag(USART3, USART_FLAG_IDLE);
	USART_ClearITPendingBit(USART3,USART_FLAG_IDLE);
	USART_ITConfig(USART3, USART_IT_IDLE, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART3_DMA();
}

void USART6_DEVICE(void)/*JUDGE*/
{
	USART6_DMA();
	
	/**USART6 GPIO Configuration
	PG14     ------> USART6_TX
	PG9     ------> USART6_RX
	*/
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;

	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOG, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART6, ENABLE);

//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, ENABLE);
//	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART6, DISABLE);
	
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_UP;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 | GPIO_Pin_9;
	GPIO_Init(GPIOG, &GPIO_InitStructure);

	GPIO_PinAFConfig(GPIOG, GPIO_PinSource14, GPIO_AF_USART6);
	GPIO_PinAFConfig(GPIOG, GPIO_PinSource9, GPIO_AF_USART6);
	
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART6, &USART_InitStructure);
  
//	USART_ClearFlag(USART6, USART_FLAG_IDLE);
//	USART_ITConfig(USART6, USART_IT_IDLE, ENABLE);

	USART_ClearFlag(USART6,USART_FLAG_TC);
	USART_ClearFlag(USART6,USART_FLAG_RXNE);
	//开启串口空闲中断&DMA
	USART_ITConfig(USART6,USART_IT_IDLE,ENABLE);
	USART_DMACmd(USART6, USART_DMAReq_Tx, ENABLE);
	USART_DMACmd(USART6, USART_DMAReq_Rx, ENABLE);
	
	NVIC_InitTypeDef NVIC_InitStructure;	

	NVIC_InitStructure.NVIC_IRQChannel = DMA2_Stream6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART6_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_Init(&NVIC_InitStructure);

	USART_Cmd(USART6, ENABLE);
}

void USART1_DEVICE(void)/*PC*/
{
	/**USART1 GPIO Configuration
	PB7     ------> USART1_RX
	PA9     ------> USART1_TX
	*/
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOB,ENABLE);
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1,ENABLE);
	
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, ENABLE);
	RCC_APB2PeriphResetCmd(RCC_APB2Periph_USART1, DISABLE);
	
	GPIO_PinAFConfig(GPIOB, GPIO_PinSource7, GPIO_AF_USART1);
	GPIO_PinAFConfig(GPIOA, GPIO_PinSource9, GPIO_AF_USART1);
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_7;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOB,&GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
	GPIO_InitStructure.GPIO_Speed = GPIO_High_Speed;
	GPIO_Init(GPIOA,&GPIO_InitStructure);
	
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_Mode = USART_Mode_Tx|USART_Mode_Rx;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_Init(USART1,&USART_InitStructure);
	
	USART_ClearFlag(USART1, USART_FLAG_IDLE);
	USART_ITConfig(USART1, USART_IT_IDLE, ENABLE);
	USART_Cmd(USART1, ENABLE);
	
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 5;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	
	USART1_DMA();
}

u8 Judge_Tx_done = 1;

uint8_t DMA_Judge_USART_Tx_Data(uint8_t* addr, uint32_t size)
{
	if(Judge_Tx_done == 0)
		return 0;
	else
	{
		Judge_Tx_done = 0;
		DMA_SetCurrDataCounter(DMA2_Stream6, size);
		DMA_MemoryTargetConfig(DMA2_Stream6, (uint32_t)addr, DMA_Memory_0);
		DMA_Cmd(DMA2_Stream6, ENABLE);				//开始DMA发送
		return 1;
	}
}
//串口发送DMA中断
void DMA2_Stream6_IRQHandler(void)
{
	if(DMA_GetITStatus(DMA2_Stream6, DMA_IT_TCIF6) == SET)
	{
		Judge_Tx_done = 1;
		DMA_Cmd(DMA2_Stream6, DISABLE);
		DMA_ClearITPendingBit(DMA2_Stream6,DMA_IT_TCIF6);
	}
}

