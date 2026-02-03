#include "judge_task.h"
#include "STM32_TIM_BASE.h"

#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "dma.h"
#include "modeswitch_task.h"
#include "comm_task.h"
#include "detect_task.h"
#include "data_packet.h"
#include "judge_rx_data.h"
#include "judge_tx_data.h"
#include "remote_ctrl.h"

extern TaskHandle_t judge_rx_Task_Handle;

UBaseType_t judge_tx_stack_surplus;
UBaseType_t judge_rx_stack_surplus;
uint8_t DATA[113] = {"AwakeLion!!!"}; // 该数组用来储存机器人交互的数据，可用户自行修改
void judge_tx_task(void *parm)
{
	uint32_t judge_wake_time = osKernelSysTick();
	// uint32_t judge_wake_time1 = osKernelSysTick();
	// uint32_t judge_wake_time2 = osKernelSysTick();
	while (1)
	{
		if (global_mode == RELEASE_CTRL) // 断开控制时刷新
			do
			{
				while (!send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID))
					; // 发送包函数
			} while (!staic_ui_refresh());
		else
			do
			{
				while (!send_packed_fifo_data(&judge_txdata_fifo, DN_REG_ID))
					; // 发送包函数
			} while (!dynamic_ui_refresh());
		vTaskDelayUntil(&judge_wake_time, 30); // 裁判系统上限30hz，且每次只能发一个包
	}
}

void judge_rx_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;

	while (1)
	{
		STAUS = xTaskNotifyWait((uint32_t)NULL,
								(uint32_t)JUDGE_UART_IDLE_SIGNAL,
								(uint32_t *)&Signal,
								(TickType_t)portMAX_DELAY);
		if (STAUS == pdTRUE)
		{
			if (Signal & JUDGE_UART_IDLE_SIGNAL)
			{
				USART_ClearFlag(USART1, USART_FLAG_IDLE);				  // 清除空闲中断标志位
				dma_buffer_to_unpack_buffer(&judge_rx_obj, UART_IDLE_IT, JUDGE_MAX_LEN); // 通过指针取址的方法把串口接收的数据放进FIFO
				unpack_fifo_data(&judge_unpack_obj, DN_REG_ID);			  // 同样再通过指针取址的方法把FIFO里的数据拿出来放进一个数组里
			}
		}
		judge_rx_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}

void DMA2_Stream1_IRQHandler(void)
{
	if(DMA_GetFlagStatus(DMA2_Stream1,DMA_FLAG_TCIF1) != RESET 
		 && DMA_GetITStatus(DMA2_Stream1,DMA_IT_TCIF1) != RESET)
	{
		dma_buffer_to_unpack_buffer(&judge_rx_obj,UART_DMA_FULL_IT, JUDGE_MAX_LEN);
		DMA_ClearFlag(DMA2_Stream1, DMA_FLAG_TCIF1);
		DMA_ClearITPendingBit(DMA2_Stream1, DMA_IT_TCIF1);
	}	
}

/*USART6 中断函数*/
void USART6_IRQHandler(void)
{
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;

	if (USART_GetFlagStatus(USART6, USART_FLAG_IDLE) != RESET // 判断是否空闲总线              USART_FLAG_TC
		&& USART_GetITStatus(USART6, USART_IT_IDLE) != RESET) // 判断是否空闲总线中断
	{
		dma_buffer_to_unpack_buffer(&judge_rx_obj, UART_IDLE_IT, JUDGE_MAX_LEN); // 通过指针取址的方法把串口接收的数据放进FIFO
		USART_ReceiveData(USART6);
		USART_ClearFlag(USART6, USART_FLAG_IDLE); // 清除空闲中断标志位
		USART_ClearITPendingBit(USART6, USART_FLAG_IDLE);
		err_detector_hook(JUDGE_SYS_OFFLINE);

		if (judge_rx_Task_Handle != NULL) // 避免任务没来得及创建就发送信号量，导致卡在断言机制中
		{
			xTaskNotifyFromISR((TaskHandle_t)judge_rx_Task_Handle,
							   (uint32_t)JUDGE_UART_IDLE_SIGNAL,
							   (eNotifyAction)eSetBits,
							   (BaseType_t *)&xHigherPriorityTaskWoken);
			/*进行上下文切换*/
			if (xHigherPriorityTaskWoken != pdFALSE)
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
		}
	}
}
