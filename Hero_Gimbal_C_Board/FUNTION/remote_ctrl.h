#ifndef _remote_ctrl_H
#define _remote_ctrl_H

#include "stm32f4xx.h"
#include "rc.h"

/*底盘控制*/
#define RC_DODGE_MODE            	(((rc.image.sw != RC_C) || ((rc.sw2 != RC_DN) && (rc.sw1 == RC_MI))) && ((rc.iw >= IW_DN) || (rc.iw <= IW_UP)))

/*射击控制*/
#define RC_SINGLE_SHOOT    			((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_DN)) || ((glb_sw.last_trig == 0) && (rc.image.trig == 1))
#define RC_SINGLE_SHOOT_AFTER  		(rc.sw1 == RC_DN && glb_sw.last_sw1 == RC_DN) || ((glb_sw.last_trig == 1) && (rc.image.trig == 1))

#define RC_CTRL_FRIC_WHEEL 			((glb_sw.last_sw1 == RC_MI) && (rc.sw1 == RC_UP)) || ( (glb_sw.last_key_r == 0) && (rc.image.key_r == 1) ) 
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
	IW_UP = -200,		//	min 364		middle 1024	拨轮
	IW_DN = 200,		//	max 1684
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
	uint8_t last_pause;
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
extern uint8_t RC_iw_flag;
void remote_ctrl(rc_info_t *rc,uint8_t *dbus_buf);
void remote_ctrl_shoot_hook(void);
void transfer_image_data_handler(rc_info_t *rc, uint8_t *p_frame);
void rc_iw_judge(void);
#endif


