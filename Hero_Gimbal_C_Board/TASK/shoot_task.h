#ifndef _shoot_task_H
#define _shoot_task_H


#include "stm32f4xx.h"

typedef enum
{
	SHOOTSTOP_MODE    	= 0, //失能
	SHOOTMAD_MODE		= 1,

}shoot_para_e;

typedef __packed struct
{
  /* shoot task relevant param */
	uint32_t rc_shoot_time; 		//遥控发射的时间计算
	uint32_t c_shoot_time; 			//键鼠发射的时间计算
	
    uint8_t shoot_cmd;				//手动控制的发射使能标志位
	uint8_t shoot_allow;			//射击热量允许
	uint8_t fric_trun_on;			//水平摩擦轮打开标志位

	uint8_t loader_fire_allow;		//拨盘发射允许
	
	uint8_t     last_para_mode; 	//模式
	uint8_t		para_mode;
	uint8_t  	allow;
	uint8_t     fric_wheel_run; 	//水平摩擦轮开关标志位	
	uint16_t    fric_wheel_spd_ref;	//水平摩擦轮参考速度
	uint32_t   	time;
	
} shoot_t;

typedef __packed struct
{
	uint16_t 	servor_id; 				//舵机ID
	uint16_t 	servor_time;			//舵机运动时间
	uint16_t 	servor_add_time;		//舵机运动时间(吊射模式)
	uint16_t 	Normal_angle;			//正常模式的舵机角度
	uint16_t	Shoot_angle;			//吊射模式的舵机角度
	
} image_t;

typedef enum
{
  SHOOT_CMD,
  FRIC_CTRL,
} shoot_type_e;

extern shoot_t   shoot;
extern float current_heat;

void shoot_ready_flag_handle(void);
void shoot_task(void *parm);

void shoot_param_init(void);
void get_last_shoot_mode(void);
static void shoot_para_ctrl(void);
static void fric_wheel_ctrl(void);
static void servor_image_handler(void);
void image_servor_init(void);
uint16_t heat_calc(void);


#endif

