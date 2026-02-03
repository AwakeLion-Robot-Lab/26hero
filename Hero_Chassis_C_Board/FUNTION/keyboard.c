#include "keyboard.h"
#include "remote_ctrl.h"
#include "STM32_TIM_BASE.h"
#include "sys_config.h"
#include "ramp.h"
#include "chassis_task.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "chassis_task.h"
#include "judge_rx_data.h" //!!!
#include "ladrc.h"
#include "modeswitch_task.h"

kb_ctrl_t km; // 键鼠控制变量

/*控制底盘朝向*/
static void chassis_face_ctrl(uint8_t normal, uint8_t sideways)
{
	if (normal)
		km.face = 0;
	else if (sideways)
		km.face = 1;
	else if ((FORWARD && FAST_SPD) != 0 || (BACK && FAST_SPD) != 0)
		km.face = 0;
}

/*控制方向*/
static void chassis_direction_ctrl(uint8_t forward, uint8_t back,
								   uint8_t left, uint8_t right)
{
	// 前后控制(前正后负)
	if (back)
	{
		km.vx = -1;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}
	else if (forward)
	{
		km.vx = 1;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}
	else
	{
		km.vx = 0;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}

	// 左右控制(左正右负)
	if (right)
	{
		km.vy = -1;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}
	else if (left)
	{
		km.vy = 1;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}
	else
	{
		km.vy = 0;
		rm.vx = 0;
		rm.vy = 0;
		rm.vw = 0;
	}

	if (chassis.direction == 0)
	{
		km.vx = -km.vx;
		km.vy = -km.vy;
		rm.vx = -rm.vx;
		rm.vy = -rm.vy;
	}
}
static void kb_enable_hook(void)
{
  if (rc.sw2 != RC_DN || rc.image.sw != RC_C)//RC_MI
	  km.kb_enable = 1;
  else
    km.kb_enable = 0;
}
void keyboard_chassis_hook(void)
{
	kb_enable_hook();
	if (km.kb_enable)
	{
		chassis_direction_ctrl(FORWARD, BACK, LEFT, RIGHT);
		chassis_face_ctrl(FACE_NOR, FACE_SIDE);
	}
	else
	{
		km.vx = 0;
		km.vy = 0;
	}
}
