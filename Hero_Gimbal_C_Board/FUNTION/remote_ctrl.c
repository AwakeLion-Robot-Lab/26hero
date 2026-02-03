 #include "remote_ctrl.h"
#include "stdlib.h"
#include "string.h"
#include "sys_config.h"
#include "STM32_TIM_BASE.h"
#include "shoot_task.h"
#include "keyboard.h"
#include "gimbal_task.h"


/*遥控接收机在底盘，故云台此部分代码是没有用的，如调试有需要烧录云台代码，把remote_ctrl()函数注释部分注释回来即可*/
rc_ctrl_t rm; //拨杆变量
sw_record_t glb_sw; //switch变量
uint8_t RC_iw_flag = 0;

void remote_ctrl(rc_info_t *rc,uint8_t *dbus_buf)
{
	rc->ch1 = (dbus_buf[0] | dbus_buf[1] << 8) & 0x07FF;
  rc->ch1 -= 1024;
  rc->ch2 = (dbus_buf[1] >> 3 | dbus_buf[2] << 5) & 0x07FF;
  rc->ch2 -= 1024;
  rc->ch3 = (dbus_buf[2] >> 6 | dbus_buf[3] << 2 | dbus_buf[4] << 10) & 0x07FF;
  rc->ch3 -= 1024;
  rc->ch4 = (dbus_buf[4] >> 1 | dbus_buf[5] << 7) & 0x07FF;
  rc->ch4 -= 1024;
  
  /* prevent remote control zero deviation */
  if(rc->ch1 <= 5 && rc->ch1 >= -5)
    rc->ch1 = 0;
  if(rc->ch2 <= 5 && rc->ch2 >= -5)
    rc->ch2 = 0;
  if(rc->ch3 <= 5 && rc->ch3 >= -5)
    rc->ch3 = 0;
  if(rc->ch4 <= 5 && rc->ch4 >= -5)
    rc->ch4 = 0;
  
  rc->sw1 = ((dbus_buf[5] >> 4) & 0x000C) >> 2;
  rc->sw2 = (dbus_buf[5] >> 4) & 0x0003;
  rc->iw = (dbus_buf[16] | dbus_buf[17] << 8) & 0x07FF;
		rc->iw -= 1024;
  if ((abs(rc->ch1) > 660) || \
      (abs(rc->ch2) > 660) || \
      (abs(rc->ch3) > 660) || \
      (abs(rc->ch4) > 660))
  {
    memset(rc, 0, sizeof(rc_info_t));
    return ;
  }
 
  rc->mouse.x = dbus_buf[6] | (dbus_buf[7] << 8); // x axis
  rc->mouse.y = dbus_buf[8] | (dbus_buf[9] << 8);
  rc->mouse.z = dbus_buf[10] | (dbus_buf[11] << 8);

  rc->mouse.l = dbus_buf[12];
  rc->mouse.r = dbus_buf[13];

  rc->kb.key_code = dbus_buf[14] | dbus_buf[15] << 8; // key borad code

  	/*一旦检测到使用dbus就断开图传链路，防止两个遥控冲突*/
	rc->image.key_l = 0;
	rc->image.key_r = 0;
	rc->image.pause = 0;
	rc->image.trig = 0;
	rc->image.sw = 0;

}

void transfer_image_data_handler(rc_info_t *rc, uint8_t *p_frame)
{
	rc->ch1 = (p_frame[2] | p_frame[3] << 8) & 0x07FF;					
	rc->ch1 -= 1024;
	rc->ch2 = (p_frame[3] >> 3 | p_frame[4] << 5) & 0x07FF;
	rc->ch2 -= 1024;
	rc->ch3 = (p_frame[6] >> 1 | p_frame[7] << 7) & 0x07FF;
	rc->ch3 -= 1024;
	rc->ch4 = (p_frame[4] >> 6 | p_frame[5] << 2 | p_frame[6] << 10) & 0x07FF;
	rc->ch4 -= 1024;

	/* prevent remote control zero deviation */
	if (rc->ch1 <= 5 && rc->ch1 >= -5)
		rc->ch1 = 0;      
	if (rc->ch2 <= 5 && rc->ch2 >= -5)
		rc->ch2 = 0;      
	if (rc->ch3 <= 5 && rc->ch3 >= -5)
		rc->ch3 = 0;      
	if (rc->ch4 <= 5 && rc->ch4 >= -5)
		rc->ch4 = 0;

	if ((abs(rc->ch1) > 660) ||
		(abs(rc->ch2) > 660) ||
		(abs(rc->ch3) > 660) ||
		(abs(rc->ch4) > 660))
	{
		memset(&rc, 0, sizeof(rc_info_t));
		return;
	}
	
	rc->image.sw = ((p_frame[7] >> 4) & 0x03);
	rc->image.pause = ((p_frame[7] >> 6) & 0x01);
	rc->image.key_l = ((p_frame[7] >> 7) & 0x01);
	rc->image.key_r = p_frame[8] & 0x01;
	rc->image.trig = ((p_frame[9] >> 4) & 0x01);
	rc->iw = -((p_frame[8] >> 1 | p_frame[9] << 7) & 0x07FF);
	rc->iw += 1024;//遥控和图传遥控滚轮反了
	if (rc->iw <= 5 && rc->iw >= -5)
		rc->iw = 0;

	rc->mouse.x = p_frame[10] | (p_frame[11] << 8); // x axis
	rc->mouse.y = p_frame[12] | (p_frame[13] << 8);
	rc->mouse.z = p_frame[14] | (p_frame[15] << 8);
	rc->mouse.l = p_frame[16] & 0x03;
	rc->mouse.r = (p_frame[16] >> 2) & 0x03;
	rc->mouse.m = (p_frame[16] >> 4) & 0x03;
	rc->kb.key_code = p_frame[17] | p_frame[18] << 8; // key borad code
	rc->CRC16 = p_frame[19] | p_frame[20] << 8;
	
	rc->sw1 = 0;
	rc->sw2 = 0;
}


//*******************************底盘*******************************************************
void rc_iw_judge(void)
{
	if(rc.iw <= IW_UP)
		RC_iw_flag = 1;
	else
		RC_iw_flag = 0;
}

//*******************************发射*******************************************************
static void horizon_fric_operation_func(uint8_t ctrl_fric)
{
  if(ctrl_fric && (global_mode != RELEASE_CTRL))
  {
    shoot.fric_wheel_run =  !shoot.fric_wheel_run ;
  }
}


static void rc_shoot_cmd(uint8_t single_fir, uint8_t single_fir_after)
{
  if(gimbal_mode != GIMBAL_TRACK_ARMOR)
  {
	if (single_fir)
	{
		shoot.c_shoot_time = HAL_GetTick();//发弹计时
		shoot.shoot_cmd   = 1;
	}
		/* 遥控部分发弹防连发保险判断 */
	if (single_fir_after && (HAL_GetTick() - shoot.c_shoot_time >= 70)) //！！！70ms后取消发弹，强制垂直摩擦轮停转
	{
		shoot.shoot_cmd   = 0;
	}
  }
  if(gimbal_mode == GIMBAL_TRACK_ARMOR)
  {
	if (single_fir_after || single_fir) 
	{
		shoot.shoot_cmd = 1;
	}
	else
	{
		shoot.shoot_cmd = 0;
	}
  
  }
}

static void kb_enable_hook(void)
{
  if (rc.sw2 != RC_DN || rc.image.sw != RC_C)//RC_MI
	  km.kb_enable = 1;
  else
	  km.kb_enable = 0;
}

void remote_ctrl_shoot_hook(void)
{
  //开摩擦轮
  horizon_fric_operation_func(RC_CTRL_FRIC_WHEEL);
  rc_shoot_cmd(RC_SINGLE_SHOOT, RC_SINGLE_SHOOT_AFTER);
  kb_enable_hook();
  if(global_mode == RELEASE_CTRL)
  {
	shoot.fric_wheel_run = 0;
	shoot.shoot_cmd      = 0;
  }
}


//*************************************************************************************************************

