#ifndef _remote_ctrl_H
#define _remote_ctrl_H

#include "stm32f4xx.h"
#include "rc.h"

enum
{
	RC_C = 0,
	RC_N = 1,
	RC_S = 2,
};
enum
{
	RC_UP = 1,
	RC_MI = 3,
	RC_DN = 2,
	IW_UP = -400,		//	min 364		middle 1024	拨轮
	IW_DN = 400,		//	max 1684
	IW_MI = 0,
};

typedef struct
{
	/*记录拨杆上一次状态*/
	uint8_t last_sw1;
	uint8_t last_sw2;
	uint16_t last_iw;
	uint8_t last_key_r;
	uint8_t last_key_l;
	uint8_t last_trig;

} sw_record_t;

typedef struct
{
	/*底盘方向*/
  float vx;
  float vy;
  float vw;
  /*云台方向*/
  float pit_v;
  float yaw_v;
} rc_ctrl_t;

extern rc_ctrl_t rm;
extern sw_record_t glb_sw;

void remote_ctrl(rc_info_t *rc,uint8_t *dbus_buf);
void remote_ctrl_chassis_hook(void);

#endif


