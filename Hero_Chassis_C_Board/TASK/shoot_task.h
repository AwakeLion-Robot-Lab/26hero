#ifndef _shoot_task_H
#define _shoot_task_H

#include "stm32f4xx.h"

typedef struct
{
	uint8_t fric_trun_on;
	uint8_t last_para_mode;
	uint8_t para_mode;
	uint8_t loader_fire_allow;
	
} shoot_t;

typedef struct
{
  /* trigger motor param */
  int32_t angle_ref;
  int32_t spd_ref; // 触发电机速度参考值

} loader_t;

typedef enum
{
  SHOOT_CMD,
  FRIC_CTRL,
} shoot_type_e;

extern shoot_t shoot;
extern loader_t loader;
extern float current_heat;

void shoot_task(void *parm);

void shoot_param_init(void);
static void shoot_bullet_handler(void);
static void turn_on_friction_wheel(int16_t lspd, int16_t rspd);
static void turn_off_friction_wheel(void);


#endif
