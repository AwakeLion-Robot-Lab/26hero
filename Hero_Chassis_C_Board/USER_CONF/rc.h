#ifndef _rc_H
#define _rc_H


#include "stm32f4xx.h"

enum
{
	Old_Remote_enable = 1,
	New_Remote_enable = 0,
};
typedef __packed struct
{
	/* 摇杆 */
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	/* 拨杆 */
	uint8_t sw1;
	uint8_t sw2;
	/* 拨轮 */
	int16_t iw;

   __packed struct
	{
		uint8_t sw;			//新图传
		uint8_t pause;		//暂停键（手册名称）
		uint8_t key_l;		//左自定义
		uint8_t key_r;		//右自定义
		uint8_t trig;		//扳机
	}image;
  /* 鼠标 */
  __packed struct
  {
    int16_t x;
    int16_t y;
    int16_t z;
  
    uint8_t l;
    uint8_t r;					
	uint8_t m;					//中键
  } mouse;
  /* 键盘 */
  __packed union
  {
    uint16_t key_code;
    __packed struct 
    {
      uint16_t W:1;
      uint16_t S:1;
      uint16_t A:1;
      uint16_t D:1;
      uint16_t SHIFT:1;
      uint16_t CTRL:1;
      uint16_t Q:1;
      uint16_t E:1;
      uint16_t R:1;
      uint16_t F:1;
      uint16_t G:1;
      uint16_t Z:1;
      uint16_t X:1;
      uint16_t C:1;
      uint16_t V:1;
      uint16_t B:1;
    } bit;
  } kb;
  uint16_t CRC16;			//CRC16_CCITT_FALSE
  uint8_t remote_flag; 		//1:老遥控 0:新遥控
  int 	iw_flag;			//1:顺时针 -1:逆时针 0:停止
} rc_info_t;

extern rc_info_t   rc;

void DMA_ReStart(void);



#endif

