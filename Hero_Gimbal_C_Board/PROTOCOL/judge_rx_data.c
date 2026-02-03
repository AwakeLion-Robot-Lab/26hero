#include "judge_rx_data.h"
#include "dma.h"
#include "string.h"
#include "data_packet.h"
#include "modeswitch_task.h"
#include "rc.h"
#include "FreeRTOSConfig.h"
/*................变量..................*/
judge_rxdata_t judge_recv_mesg;
//------------------------------------------------------------------------------------------------------------------//
uart_dma_rxdata_t image_rx_obj;
image_unpack_data_t image_unpack_obj;

static SemaphoreHandle_t image_rxdata_mutex;


fifo_s_t  image_rxdata_fifo;
static uint8_t   image_rxdata_buf[IMAGE_RX_FIFO_BUFLEN];

void transfer_image_rx_param_init(void)
{
  /* create the judge_rxdata_mutex mutex  */  
  image_rxdata_mutex = xSemaphoreCreateMutex();
//    

  /* judge data fifo init fifo存储器--先进先出*/
  fifo_s_init(&image_rxdata_fifo, image_rxdata_buf, IMAGE_RX_FIFO_BUFLEN, image_rxdata_mutex); //添加judge_rxdata_fifo的互斥量

  
  /* initial judge data dma receiver object 初始裁判数据dma接收对象 */
  image_rx_obj.dma_stream = DMA2_Stream1;
  image_rx_obj.data_fifo = &image_rxdata_fifo;
  image_rx_obj.buff_size = IMAGE_RX_FIFO_BUFLEN;
  image_rx_obj.buff[0] = image_buf[0];
  image_rx_obj.buff[1] = image_buf[1];

  /* initial judge data unpack object 初始裁判数据解包对象 */
  image_unpack_obj.data_fifo = &image_rxdata_fifo;
  image_unpack_obj.index = 0;
  image_unpack_obj.unpack_step = STEP_HEADER_SOF1;	
}
