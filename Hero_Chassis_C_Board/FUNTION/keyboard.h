#ifndef _keyboard_H
#define _keyboard_H

#include "stm32f4xx.h"

/* control key definition */
//      direction  key
#define FORWARD (rc.kb.bit.W)
#define BACK (rc.kb.bit.S)
#define LEFT (rc.kb.bit.A)
#define RIGHT (rc.kb.bit.D)
//      speed      key
#define FAST_SPD (rc.kb.bit.SHIFT)
#define SLOW_SPD (rc.kb.bit.CTRL)
//		chassis	turn key
#define FACE_NOR (rc.kb.bit.CTRL)
#define FACE_SIDE (rc.kb.bit.Z)

/*小陀螺模式*/
#define KB_DODGE_CTRL (rc.kb.bit.E)                  //有用
#define KB_DODGE_CTRL_CLOSE (rc.kb.bit.CTRL)         //有用

typedef struct
{
  uint8_t kb_enable; //

  float vx_limit_speed;
  float vy_limit_speed;
  float vw_limit_speed;

  /*底盘朝向*/
  uint8_t face;
  /*底盘方向*/
  float vx;
  float vy;
  float vw;
  /*云台方向*/
  float pit_v;
  float yaw_v;
} kb_ctrl_t;

extern kb_ctrl_t km;

void keyboard_global_hook(void);
void keyboard_chassis_hook(void);
static void kb_enable_hook(void);

#endif
