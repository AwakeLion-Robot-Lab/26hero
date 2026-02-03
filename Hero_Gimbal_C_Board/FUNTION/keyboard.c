#include "keyboard.h"
#include "remote_ctrl.h"
#include "STM32_TIM_BASE.h"
#include "sys_config.h"
#include "ramp.h"
#include "gimbal_task.h"
#include "shoot_task.h"
#include "judge_rx_data.h" //!!!
#include "ladrc.h"
#include "modeswitch_task.h"
#include "stdlib.h" //abs()函数

kb_ctrl_t km; // 键鼠控制变量

uint8_t gimbal_turn_flag;
uint8_t image_shoot_flag;	//图传吊射模式标志位
int8_t image_add_flag; 		//图传抬升标志位
uint8_t glass_flag;			
extern int direction_change;

void get_mouse_status(MOUSE_STATUS *status, uint8_t mouse)
{
	switch (*status)
	{
	case MOUSE_RELEASE:
	{
		if (mouse)
			*status = MOUSE_PRESS;
		else
			*status = MOUSE_RELEASE;
	}
	break;

	case MOUSE_PRESS:
	{
		if (mouse)
			*status = MOUSE_DONE;
		else
			*status = MOUSE_RELEASE;
	}
	break;

	case MOUSE_DONE:
	{
		if (mouse)
		{
			*status = MOUSE_ONCE;
			if (status == &km.l_mouse_sta)
				km.l_cnt = HAL_GetTick();
			else
				km.r_cnt = HAL_GetTick();
		}
		else
			*status = MOUSE_RELEASE;
	}
	break;

	case MOUSE_ONCE:
	{
		if (mouse)
		{
			if (status == &km.l_mouse_sta)
			{
				if (HAL_GetTick() - km.l_cnt > 100)
					*status = MOUSE_LONG;
			}
			else
			{
				if (HAL_GetTick() - km.r_cnt > 100)
					*status = MOUSE_LONG;
			}
		}
		else
			*status = MOUSE_RELEASE;
	}
	break;

	case MOUSE_LONG:
	{
		if (!mouse)
		{
			*status = MOUSE_RELEASE;
		}
	}
	break;

	default:
	{
	}
	break;
	}
}

/*WASD控制云台*/
static void gimbal_ctrl(uint8_t forward, uint8_t back,
						uint8_t left, uint8_t right)
{
	if (forward || back || left || right)
	{
		if (forward)
			km.pit_v = 1;
		else if (back)
			km.pit_v = -1;
		else
			km.pit_v = 0;
		
		if (left)
			km.yaw_v = -1;
		else if (right)
			km.yaw_v = 1;
		else
			km.yaw_v = 0;
	}
	else
	{
		km.pit_v = 0;
		km.yaw_v = 0;
	}
}
uint32_t turn_time;
uint32_t turn_delay_t;
uint8_t gimbal_turn_ready;
uint8_t first_once = 1;
/*云台掉头*/
void gimbal_turn_back(uint8_t turn_back_key)
{
	if((turn_back_key && first_once))
	{
		gimbal_turn_flag = 1;
		turn_time = HAL_GetTick();
		first_once = 0;
	}
	else
	{
		gimbal_turn_flag = 0;
	}
	if(gimbal_turn_flag)
	{
		gimbal_turn_ready = 1;
	}
	if( (turn_delay_t>1000) && !gimbal_turn_flag)
	{
		gimbal_turn_ready = 0;
		first_once = 1;
		
	}
	turn_delay_t = HAL_GetTick() - turn_time;
	
}

void keyboard_global_hook(void)
{
	if (km.kb_enable)
	{
		get_mouse_status(&km.l_mouse_sta, rc.mouse.l);
		get_mouse_status(&km.r_mouse_sta, rc.mouse.r);
	}
	
}


static void gimbal_speed_ctrl()
{
	static uint32_t last_time, now_time;
	now_time = HAL_GetTick();
	
	if(km.r_mouse_sta == MOUSE_LONG && km.last_r_mouse_sta != MOUSE_LONG )
	{
		rc.mouse.y = 0;
		rc.mouse.x = 0;
	}
	km.pit_v = rc.mouse.y * ((now_time - last_time) * 0.001f);
	km.yaw_v = rc.mouse.x * ((now_time - last_time) * 0.001f);
	
	last_time = now_time;
	km.last_r_mouse_sta = km.r_mouse_sta;
}

void keyboard_gimbal_hook(void)
{
	if (km.kb_enable)
	{
		gimbal_speed_ctrl();
		gimbal_turn_back(KB_GIMBAL_TURN_BACK);
	}
	else
	{
		km.pit_v = 0;
		km.yaw_v = 0;
	}
}


uint8_t match_status;
static void shoot_cmd_ctrl(uint8_t shoot_cmd, uint8_t c_shoot_cmd)
{
	match_status = judge_recv_mesg.game_state.game_type;

	if (match_status == 4)
	{
		if (judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit - heat_calc() >= 110)
			shoot.shoot_allow = 1;
		else
			shoot.shoot_allow = 0;
	}
	else
	{
		shoot.shoot_allow = 1;
	}

	if (judge_recv_mesg.power_heat_data.shooter_42mm_barrel_heat == 100)
	{
		if ((shoot_cmd || c_shoot_cmd) && (judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit - heat_calc() >= 110) && (HAL_GetTick() - shoot.c_shoot_time >= 800)) 
		{
			shoot.c_shoot_time = HAL_GetTick();
			shoot.shoot_cmd = 1;
		}
	}
	else
	{
		if ((shoot_cmd || c_shoot_cmd) && (judge_recv_mesg.game_robot_state.shooter_barrel_heat_limit - heat_calc() >= 110))
		{
			shoot.c_shoot_time = HAL_GetTick();
			shoot.shoot_cmd = 1; 
		}
	}

	if (HAL_GetTick() - shoot.c_shoot_time >= 70)
	{
		shoot.shoot_cmd = 0;
	}
}

static void firc_ctrl(uint8_t firc_open, uint8_t firc_close)
{
	if (KB_OPEN_FRIC_WHEEL && (global_mode != RELEASE_CTRL))
		shoot.fric_wheel_run = 1;
	else if (KB_CLOSE_FIRC_WHEEL)
		shoot.fric_wheel_run = 0;
}

static void Image_pit_ctrl(uint8_t image_normal,uint8_t image_shoot)
{
	if(image_normal || gimbal_mode != GIMBAL_SEPARATE_MODE)
		image_shoot_flag = 0;
	else if(image_shoot )
		image_shoot_flag = 1;
}
static void Image_pit_add(uint8_t image_add_key ,uint8_t image_sub_key )
{
	if(image_add_key)
		image_add_flag = 1;
	else if(image_sub_key)
		image_add_flag = -1;
	else
		image_add_flag = 0;
}
static void Glass_ctrl(uint8_t glass_on,uint8_t glass_off)
{
	if(glass_on)
		glass_flag = 1;
	if(glass_off || gimbal_mode != GIMBAL_SEPARATE_MODE)
		glass_flag = 0;
}


void keyboard_shoot_hook(void)
{
		firc_ctrl(KB_OPEN_FRIC_WHEEL, KB_CLOSE_FIRC_WHEEL);
		shoot_cmd_ctrl(KB_SINGLE_SHOOT, KB_CONTINUE_SHOOT);

}

void Keyboard_gimbal_setpart_hook(void)
{
	if(km.kb_enable)
	{
		gimbal_ctrl(FORWARD, BACK, LEFT, RIGHT);
		gimbal_turn_back(KB_GIMBAL_TURN_BACK);
	}
	else
	{
		km.pit_v = 0;
		km.yaw_v = 0;
	}
}
//图传倍镜控制
void Keyboard_image_glass_ctrl_hook(void)
{
	if(km.kb_enable)
	{
		Image_pit_ctrl(KB_IMAGE_NORMAL,KB_IMAGE_SHOTPOST);
		Glass_ctrl(KB_GLASS_CTRL,KB_GLASS_CLOSE);
		Image_pit_add(KB_IMAGE_ADDPOST,KB_IMAGE_SUBPOST);
	}
	else
	{
		image_shoot_flag = 0;
		glass_flag = 0;
	}
}
