#include "shoot_task.h"
#include "STM32_TIM_BASE.h"
#include "keyboard.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "usart.h"
#include "comm_task.h"
#include "modeswitch_task.h"
#include "detect_task.h"
#include "pc_rx_data.h"
#include "string.h"
#include "sys_config.h"
#include "math.h"
#include "pid.h"
#include "bsp_can.h"
#include "judge_rx_data.h"
#include "pc_task.h"

extern TaskHandle_t can_msg_send_Task_Handle;
UBaseType_t shoot_stack_surplus;

uint8_t shoot_ready;	  // 弹丸是否上膛判断
uint32_t shoot_last_time; // 记录上一次发弹时间
//*************************************************************************************
/* 摩擦轮转速 */
uint16_t threeteen_fric_speed = 3500; //	15.4-15.9m/s模式射速 前45a后45a

/*倍镜开关角度*/
#define Glass_OFF_angle 	190
#define Glass_ON_angle 		81

/*摩擦轮pid*/
float fric_pid[Fric_Wheel_Count] = {10, 0.003f, 0.01};

//*************************************************************************************
float current_heat = 0;   		//当前热量
float shoot_already_flag =0;	//发射完成记录热量标志位

shoot_t shoot;
//图传舵机初始化
image_t image_servor;
//即时响应标志
uint32_t shoot_fin = 1;

// 拨盘发射控制
static void shoot_bullet_handler(void)
{
	if (shoot.shoot_allow && shoot.fric_trun_on)
	{
		//非自瞄模式
		if (shoot.shoot_cmd && shoot_fin == 1 && gimbal_mode != GIMBAL_TRACK_ARMOR)
		{
			shoot.loader_fire_allow = 1;
			shoot_last_time = HAL_GetTick();
			shoot_fin = 0;
		}
		//自瞄模式下
		else	
		{
			//视觉控制发弹判断
			if (gimbal_mode == GIMBAL_TRACK_ARMOR 
				&& pc_recv_mesg.mode_Union.info.fire 
				&& pc_recv_mesg.mode_Union.info.visual_valid && shoot_fin == 1 
				&& shoot.para_mode != SHOOTSTOP_MODE  		 && shoot.shoot_cmd )
			{
				shoot.loader_fire_allow = 1;
				shoot_fin = 0;
				shoot_last_time = HAL_GetTick();
			}
			else
			{
				shoot.loader_fire_allow = 0;
			}
		}
	}
	else
	{
		shoot.loader_fire_allow = 0;
	}
	if (HAL_GetTick() - shoot_last_time > 200) // 偶尔编码位置丢失摩擦轮会一直转，一段时间后速度给0
	{
		shoot_fin = 1;
	}
}	

// 42mm轻触开关检测->控制底盘拨盘旋转与垂直摩擦轮通断
void shoot_ready_flag_handle(void)
{
	if (GPIO_ReadInputDataBit(GPIOE, LED_GREEN) == Bit_SET)
	{
		GPIO_SetBits(GPIOH, LED_BLUE | LED_GREEN);
		GPIO_ResetBits(GPIOH, LED_RED); 			// shoot_ready = 1 亮蓝灯+绿灯 = 青色
		shoot_ready = 1;
		shoot_already_flag =1;
	}
	else if (GPIO_ReadInputDataBit(GPIOE, LED_GREEN) == Bit_RESET)
	{
		shoot_ready = 0;
		GPIO_SetBits(GPIOH, LED_GREEN);
		GPIO_ResetBits(GPIOH, LED_BLUE | LED_RED); 	// IMU初始化完成亮绿灯11
		if(shoot_already_flag ==1)
		{
			current_heat += 100;
			shoot_already_flag=0;
		}
	}
}
/* 倍镜舵机控制 */
uint32_t Delay_time_glass = 0;
static void servor_glass_handle(void)
{
	static uint16_t pulse_width;
	// off
	if (glass_flag == 0 && HAL_GetTick() - Delay_time_glass > 500)
	{
		pulse_width = (uint16_t)(0.5 * 20000 / 20 + Glass_OFF_angle * 20000 / 20 / 90);
		Delay_time_glass = HAL_GetTick();
	}
	else if (glass_flag == 1 && HAL_GetTick() - Delay_time_glass > 500)
	{
		pulse_width = (uint16_t)(0.5 * 20000 / 20 + Glass_ON_angle * 20000 / 20 / 90);
		Delay_time_glass = HAL_GetTick();
	}
	// 设置定时器的捕获比较寄存器来产生所需的PWM占空比
	TIM_SetCompare3(TIM8, pulse_width); // 接c板的C7位置为PWM,舵机黄色线为信号线,倍镜上电初始角度在tim.c里面设置
}

/* 数字转字符串 */
static void Num_To_string(uint16_t temp, char *string, uint16_t i_max) 
{
	for(int i=i_max;i >= 0;i--)
	{
		string[i] = temp % 10 + '0';
		temp /= 10;
	}
	string[i_max+1] = '\0';
}
char id_string[4];
char pwm_string[5];
char time_string[5];
/** 
* @brief 函数功能描述:图传总线舵机的控制
* @param id 	描述: 舵机id号(默认000)
* @param angle 	描述: 舵机指定角度
* @param time 	描述: 舵机到指定角度的时间(ms)
* @return 返回值描述:无
**/
static void servor_image_ctrl(uint8_t id, uint16_t angle, uint16_t time)
{
	//pwm 500~2500 -> 0~270°
	uint16_t pwm = 1.0*angle/270 *2000+500;
	Num_To_string(id,id_string,2);
	Num_To_string(pwm,pwm_string,3);
	Num_To_string(time,time_string,3);

	Usart_SendByte(USART1,'#');
	Usart_SendString(USART1,id_string);
	Usart_SendByte(USART1,'P');
	Usart_SendString(USART1,pwm_string);
	Usart_SendByte(USART1,'T');
	Usart_SendString(USART1,time_string);
	Usart_SendByte(USART1,'!');
}
//图传舵机参数初始化
void image_servor_init(void)
{
	image_servor.servor_id 			= 000;
	image_servor.servor_time 		= 500;
	image_servor.servor_add_time 	= 0;
	image_servor.Normal_angle 		= 81;
	image_servor.Shoot_angle 		= 115;
	
	uint8_t init_flag = 1;
	
	if(init_flag)
//		Usart_SendString(USART1,"#000PMOD1!");
		servor_image_ctrl(000,81,500);
	init_flag = 0;
}
/*图传角度控制*/
static void servor_image_handler(void)
{
	//执行一次标志位,进入if一次
	static uint8_t first_once_flag = 1;
	static uint8_t first_onceadd_flag = 1;
	
	if(first_once_flag && !image_shoot_flag)
	{
		servor_image_ctrl(image_servor.servor_id,image_servor.Normal_angle,image_servor.servor_time);
		first_once_flag = 0;
	}
	else if(!first_once_flag && image_shoot_flag)
	{
		servor_image_ctrl(image_servor.servor_id,image_servor.Shoot_angle,image_servor.servor_time);
		first_once_flag = 1;
	}
	//吊射模式下操作手对图传控制 按键:F↑V↓
	if(image_add_flag != 0 && image_shoot_flag)
	{
		if(image_add_flag == 1 && first_onceadd_flag)
		{
			first_onceadd_flag = 0;
			//舵机角度的加减与实际角度相关
			image_servor.Shoot_angle -= 2;
			if(image_servor.Shoot_angle <= 81) image_servor.Shoot_angle = 81;
			servor_image_ctrl(image_servor.servor_id,image_servor.Shoot_angle,image_servor.servor_add_time);
		}
		else if(image_add_flag == -1 && first_onceadd_flag)
		{
			first_onceadd_flag = 0;
			
			image_servor.Shoot_angle += 2;
			if(image_servor.Shoot_angle >= 140) image_servor.Shoot_angle = 140;
			servor_image_ctrl(image_servor.servor_id,image_servor.Shoot_angle,image_servor.servor_add_time);
		}
	}		
	else if(!image_add_flag) //在图传没有抬升时标志位至一
		first_onceadd_flag = 1;
}

void shoot_task(void *parm)
{
	uint32_t Signal;
	BaseType_t STAUS;

	while (1)
	{
		STAUS = xTaskNotifyWait((uint32_t)NULL,
								(uint32_t)INFO_GET_SHOOT_SIGNAL,
								(uint32_t *)&Signal,
								(TickType_t)portMAX_DELAY);
		if (STAUS == pdTRUE)
		{
			if (Signal & INFO_GET_SHOOT_SIGNAL)
			{
					/*水平摩擦轮*/
					for (int i = 0; i < Fric_Wheel_Count; i++)
					{
						PID_Struct_Init(&pid_fric[i], fric_pid[0], fric_pid[1], fric_pid[2], 14500, 800, DONE);
					}
				
					shoot_bullet_handler();		// 拨盘发射控制
					shoot_para_ctrl();			// 射击速度选择
					fric_wheel_ctrl();			// 判断是否启动摩擦轮
					servor_glass_handle();		// 倍镜控制
					servor_image_handler();		// 图传控制
					
					if (!shoot.fric_wheel_run && (gimbal_mode != GIMBAL_TRACK_ARMOR))
					{
						
						shoot_last_time = 0;
						pc_recv_mesg.mode_Union.info.fire = 0;
						pc_recv_mesg.mode_Union.info.visual_valid = 0;

						for (int i = 0; i < Fric_Wheel_Count; i++)
						{
							pid_fric[i].iout = 0;
							pid_fric[i].iterm = 0;
							pid_fric[i].out = 0;
						}
					}
					get_last_shoot_mode();

				xTaskGenericNotify((TaskHandle_t)can_msg_send_Task_Handle,
								   (uint32_t)SHOT_MOTOR_MSG_SIGNAL,
								   (eNotifyAction)eSetBits,
								   (uint32_t *)NULL);
			}
		}

		shoot_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
	}
}

/*获取上一次的射击模式*/
void get_last_shoot_mode(void)
{
	shoot.last_para_mode = shoot.para_mode;
}

/*摩擦轮转速赋值，可在这里写动态射速*/
static void shoot_para_ctrl(void)
{
	shoot.fric_wheel_spd_ref = threeteen_fric_speed;
}
/*打开水平摩擦轮*/
static void Set_friction_wheel(int16_t spd)
{	
	// 右上
	pid_calc(&pid_fric[RIGHT_FRIC_UP], moto_fric[RIGHT_FRIC_UP].speed_rpm, spd);
	glb_cur.fric_cur[RIGHT_FRIC_UP] = pid_fric[RIGHT_FRIC_UP].out;
	// 左上
	pid_calc(&pid_fric[LEFT_FRIC_UP], moto_fric[LEFT_FRIC_UP].speed_rpm, spd);
	glb_cur.fric_cur[LEFT_FRIC_UP] = pid_fric[LEFT_FRIC_UP].out;
	// 下
	pid_calc(&pid_fric[FRIC_DOWN], moto_fric[FRIC_DOWN].speed_rpm, spd);
	glb_cur.fric_cur[FRIC_DOWN] = pid_fric[FRIC_DOWN].out;
}
/*摩擦轮控制*/
static void fric_wheel_ctrl(void)
{
	if (shoot.fric_wheel_run)
	{
		Set_friction_wheel(shoot.fric_wheel_spd_ref);
		shoot.fric_trun_on = 1;
	}
	else
	{
		Set_friction_wheel(0);
		shoot.fric_trun_on = 0;
	}
}

uint16_t heat_calc(void)
{
	static uint32_t fire_time;
	static uint16_t delta_time;
	if(current_heat == 0)
	{
		fire_time = osKernelSysTick();
	}
	delta_time = (osKernelSysTick() - fire_time) / 100;
	current_heat -= (float)((judge_recv_mesg.game_robot_state.shooter_barrel_cooling_value/10.0f)*delta_time);

	fire_time += 100 * delta_time;
	if(current_heat <= 0)
		current_heat = 0;
	return current_heat;

}

/*发射机构初始化*/
void shoot_param_init(void)
{
	memset(&shoot, 0, sizeof(shoot_t));

	shoot.para_mode = SHOOTMAD_MODE;

	/*水平摩擦轮*/
	for (int i = 0; i < Fric_Wheel_Count; i++)
	{
		PID_Struct_Init(&pid_fric[i], fric_pid[0], fric_pid[1], fric_pid[2], 14500, 500, INIT);
	}
	
	for (int i = 0; i < Fric_Wheel_Count; i++)
	{
		pid_fric[i].iout = 0;
		pid_fric[i].iterm = 0;
	}
}
