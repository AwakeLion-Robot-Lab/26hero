#ifndef _usart_H
#define _usart_H

#include "stm32f4xx.h"


void USART1_DEVICE(void);
void USART3_DEVICE(void);
void USART6_DEVICE(void);

uint8_t DMA_Judge_USART_Tx_Data(uint8_t* addr, uint32_t size);

#endif 

