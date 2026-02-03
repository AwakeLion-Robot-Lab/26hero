#include "judge_tx_data.h"
#include "dma.h"
#include "judge_rx_data.h"
#include "string.h"
#include "shoot_task.h"
#include "remote_ctrl.h"
#include "chassis_task.h"
#include "modeswitch_task.h"
#include "gimbal_task.h"
#include "math.h"
#include "pc_rx_data.h"
#include "keyboard.h"

/* UI内容，每一个UI一个单独的函数然后放到图层函数中
静态：射击线、自瞄识别范围-图层0
动态：
1.简单的：Pitch角度、电容容量百分比、部署可发射数-图层1
2.复杂的：轻触开关-图层1、底盘状态-图层3、摩擦轮状态-图层3、动态瞄准线-图层4
*/

/*以下数据均是需要看实际显示效果修改的，所以在绘图时可以先用它赋值给结构体，便于我们在debug中改变结构体的
	值，等合适的值确定下来，再将这个确定的值以常数赋值给结构体*/
#if debug
judge_debug_t Judge_Debug = {0};
#endif

extern uint8_t shoot_ready;
extern chassis_wheel_t chassis;
judge_txdata_t judge_send_mesg; // 裁判系统发送结构体，用户自定义发送的数据打包在里面，到时候可以在debug里查看与修改
static SemaphoreHandle_t judge_txdata_mutex;
fifo_s_t judge_txdata_fifo; // 用来暂时存放数据的邮箱
static uint8_t judge_txdata_buf[JUDGE_TX_FIFO_BUFLEN];
// extern float cap_store;
void judgement_tx_param_init(void)
{
	/* create the judge_rxdata_mutex mutex  */
	judge_txdata_mutex = xSemaphoreCreateMutex();

	/* judge data fifo init */
	fifo_s_init(&judge_txdata_fifo, judge_txdata_buf, JUDGE_TX_FIFO_BUFLEN, judge_txdata_mutex);
}

// 客户端自定义UI界面 ！！！！！！！！！！！！！！！！！！！！！！！

uint8_t current_robot_id;
uint16_t receiver_ID;

uint8_t i = 0;

double last_value[7] = {0}; // 记录动态UI需要实时刷新部分的上一次值

// 部署模式参数
uint8_t lobshot_launchable_num;	   // 全局部署模式可发射吊射数量
uint8_t lobshot_launched_num;	   // 部署模式已经发射吊射数量
uint8_t last_launchable_shoot_num; // 上一时刻可发射数量
uint8_t last_base_HP;

static uint16_t Student_ID_Swich_Fun(uint8_t current_robot_id) // 学生端
{
	uint16_t receiver_ID = 0;
	switch (current_robot_id)
	{
	// red robot
	case STUDENT_RED_HERO_ID:
	case STUDENT_RED_ENGINEER_ID:
	case STUDENT_RED_AERIAL_ID:
	case STUDENT_RED_SENTRY_ID:
	case STUDENT_RED_INFANTRY3_ID:
	case STUDENT_RED_INFANTRY4_ID:
	case STUDENT_RED_INFANTRY5_ID:
	{
		receiver_ID = STUDENT_RED_SENTRY_ID;
	}
	break;

	// blue robot
	case STUDENT_BLUE_HERO_ID:
	case STUDENT_BLUE_ENGINEER_ID:
	case STUDENT_BLUE_AERIAL_ID:
	case STUDENT_BLUE_SENTRY_ID:
	case STUDENT_BLUE_INFANTRY3_ID:
	case STUDENT_BLUE_INFANTRY4_ID:
	case STUDENT_BLUE_INFANTRY5_ID:
	{
		receiver_ID = STUDENT_BLUE_SENTRY_ID;
	}
	break;
	}
	return receiver_ID;
}
static uint16_t Client_ID_Swich_Fun(uint8_t current_robot_id) // 客户端
{
	uint16_t receiver_ID = 0;
	switch (current_robot_id)
	{
	// red robot
	case STUDENT_RED_HERO_ID:
	{
		receiver_ID = RED_HERO_CLIENT_ID;
	}
	break;
	// blue robot
	case STUDENT_BLUE_HERO_ID:
	{
		receiver_ID = BLUE_HERO_CLIENT_ID;
	}
	break;
	}
	return receiver_ID;
}
static uint16_t get_base_HP(uint8_t current_robot_id)
{
	switch (Student_ID_Swich_Fun(current_robot_id))
	{
	case STUDENT_RED_SENTRY_ID:
		return judge_recv_mesg.game_robot_HP.red_base_HP;
	// case STUDENT_BLUE_SENTRY_ID:
	// 	return judge_recv_mesg.game_robot_HP.blue_base_HP;
	default:
		return judge_recv_mesg.game_robot_HP.blue_base_HP;
	}
}
uint8_t get_shoot_num(void) // 计算部署可发射数量
{
	if (judge_recv_mesg.game_state.game_progress == 4)
	{
		lobshot_launchable_num = (420 - judge_recv_mesg.game_state.stage_remain_time) / 20 + 2;
	}
	if (chassis.separt_ctrl /*&&(judge_recv_mesg.game_robot_state.power_management_chassis_output==0)*/)
	{
		// if ((last_launchable_shoot_num - judge_recv_mesg.bullet_remaining.projectile_allowance_42mm > 0) && (lobshot_launchable_num - lobshot_launched_num > 0))
		// {
		// 	lobshot_launched_num++;
		// 	last_launchable_shoot_num = judge_recv_mesg.bullet_remaining.projectile_allowance_42mm;
		// }
		if (receiver_ID == RED_HERO_CLIENT_ID)
		{
			if (last_base_HP - judge_recv_mesg.game_robot_HP.red_base_HP >= 300)
			{
				lobshot_launched_num++;
				last_base_HP = get_base_HP(receiver_ID);
			}
		}
	}
	if (lobshot_launchable_num - lobshot_launched_num <= 0)
		return 0;
	else
		return (lobshot_launchable_num - lobshot_launched_num);
}

// 己方机器人通信
void judgement_client_packet_pack(uint8_t *p_data)
{
	uint8_t current_robot_id = judge_recv_mesg.game_robot_state.robot_id;
	uint16_t receiver_ID = Student_ID_Swich_Fun(current_robot_id);

	judge_send_mesg.student_interactive_header_data.data_cmd_id = RobotCommunication;
	judge_send_mesg.student_interactive_header_data.sender_id = (uint16_t)current_robot_id;
	judge_send_mesg.student_interactive_header_data.receiver_id = receiver_ID;
	// 将自定义的数据复制到发送结构体中
	memcpy(&judge_send_mesg.student_interactive_header_data.user_data[0], p_data, sizeof(judge_send_mesg.student_interactive_header_data.user_data));
	// 该函数的功能为将需要发送的数据打包，便于下一步通过串口3发送给裁判系统
	data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID,
					 (uint8_t *)&judge_send_mesg.student_interactive_header_data,
					 STUDENT_DATA_LENGTH, DN_REG_ID);
}

/* UI结构体部分成员赋值 */
static void grapic_struct_assignment_fun(void *struct_ptr, uint16_t data_cmd_id)
{
	uint16_t *fields = (uint16_t *)struct_ptr;
	fields[0] = data_cmd_id;
	fields[1] = (uint16_t)current_robot_id,
	fields[2] = receiver_ID;
}
/*函数功能：交互图形结构体初始化
 * 参数里面的细节请看裁判系统串口手册，不同的图形类型作用是不一样的
 * 参数：figure：交互图形结构体指针
 * 参数：num：交互图形结构体数组下标
 * 参数：figure_name：图形名称
 * 参数：operate_tpye：操作类型
 * 参数：figure_tpye：图形类型
 * 参数：layer：图层
 * 参数：color：颜色
 * 参数：details_a：细节a
 * 参数：details_b：细节b
 * 参数：width：宽度
 * 参数：start_x：起始x坐标
 * 参数：start_y：起始y坐标
 * 参数：details_c：细节c
 * 参数：details_d：细节d
 * 参数：details_e：细节e
 * 返回值：无
 */
static void grapic_data_init_fun(interaction_figure_t *figure, uint8_t figure_name, uint32_t operate_tpye,
								 uint32_t figure_tpye, uint32_t layer, uint32_t color, uint32_t details_a, uint32_t details_b, uint32_t width,
								 uint32_t start_x, uint32_t start_y, uint32_t details_c, uint32_t details_d, uint32_t details_e)
{
	figure->figure_name[0] = figure_name;
	figure->operate_tpye = operate_tpye;
	figure->figure_tpye = figure_tpye;
	figure->layer = layer;
	figure->color = color;
	figure->details_a = details_a;
	figure->details_b = details_b;
	figure->width = width;
	figure->start_x = start_x;
	figure->start_y = start_y;
	figure->details_c = details_c;
	figure->details_d = details_d;
	figure->details_e = details_e;
}
/*画浮点数调用下面的函数*/
// static void grapic_float_fun(interaction_figure_t *figure, uint8_t figure_name, uint32_t operate_tpye, uint32_t layer,
// 							 uint32_t color, uint32_t start_x, uint32_t start_y)
// {
// 	grapic_data_init_fun(figure, figure_name, operate_tpye, Character,
// 						 layer, color, 20, 12, 3, start_x, start_y, 0, 0, 0);
// }
/*画字符调用下面的函数*/
static void grapic_string_fun(interaction_figure_t *figure, uint8_t figure_name, uint32_t operate_tpye, uint32_t layer,
							  uint32_t color, uint32_t string_length, uint32_t start_x, uint32_t start_y)
{
	grapic_data_init_fun(figure, figure_name, operate_tpye, Character,
						 layer, color, 15, string_length, 3, start_x, start_y, 0, 0, 0);
}
/*画直线调用下面的函数*/
static void grapic_line_fun(interaction_figure_t *figure, uint8_t figure_name, uint32_t operate_tpye, uint32_t layer,
							uint32_t color, uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y)
{
	grapic_data_init_fun(figure, figure_name, operate_tpye, Straight_line,
						 layer, color, 0, 0, width, start_x, start_y, 0, end_x, end_y);
}
/*画圆调用下面的函数*/
static void grapic_circle_fun(interaction_figure_t *figure, uint8_t figure_name, uint32_t operate_tpye, uint32_t layer,
							  uint32_t color, uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t Radius)
{
	grapic_data_init_fun(figure, figure_name, operate_tpye, Circle,
						 layer, color, 0, 0, width, start_x, start_y, Radius, 0, 0);
}
/*画矩形调用下面的函数*/
static void grapic_rectangle_fun(interaction_figure_t *figure, uint8_t figure_name, uint32_t operate_tpye, uint32_t layer,
								 uint32_t color, uint32_t width, uint32_t start_x, uint32_t start_y, uint32_t end_x, uint32_t end_y)
{
	grapic_data_init_fun(figure, figure_name, operate_tpye, Rectangle,
						 layer, color, 0, 0, width, start_x, start_y, 0, end_x, end_y);
}
static void shoot_line(void)
{
	grapic_struct_assignment_fun(&judge_send_mesg.layer0_shoot_line, Client_Draw_Seven_Graph);
//	// 弹丸出射点
//	grapic_circle_fun(&judge_send_mesg.layer0_shoot_line.interaction_figure[0],
//					  circle5, Add, SHOOT_LINE_LAYER, Yellow, 1, 952, 594 , 6);
	// 弹道
	grapic_rectangle_fun(&judge_send_mesg.layer0_shoot_line.interaction_figure[1],
						 rectangle0, Add, SHOOT_LINE_LAYER, Yellow, 1, 946, 800, 958, 150);
	grapic_line_fun(&judge_send_mesg.layer0_shoot_line.interaction_figure[2],
					  line_5s, Add, SHOOT_LINE_LAYER, Yellow, 1, 1120, 150, 1120 ,800);
//	// 前哨-6米
//	grapic_circle_fun(&judge_send_mesg.layer0_shoot_line.interaction_figure[2],
//					  circle6, Add, SHOOT_LINE_LAYER, Redblue, 1, 952, 494, 6);
	// 15米基地 绿色
	grapic_line_fun(&judge_send_mesg.layer0_shoot_line.interaction_figure[3],
					  line_1s, Add, SHOOT_LINE_LAYER, Green, 3, 946, 242, 958 ,242);
	// 17米基地 紫色
	grapic_line_fun(&judge_send_mesg.layer0_shoot_line.interaction_figure[4],
					  line_3s, Add, SHOOT_LINE_LAYER, Amaranth, 3, 946, 208, 958 ,208);
	 

}
 static void field_of_view_line(void)
 {	
	 grapic_struct_assignment_fun(&judge_send_mesg.layer0_field_of_view_line, Client_Draw_Two_Graph);
	 //基地10.5米 (紫红)
	 grapic_line_fun(&judge_send_mesg.layer0_field_of_view_line.interaction_figure[0],
					 line_2s, Add, FIELD_OF_VIEW_LINE_LAYER, Amaranth, 3, 946, 455, 958 ,455);
 	
// 	grapic_rectangle_fun(&judge_send_mesg.layer0_field_of_view_line.interaction_figure[0],
// 						 Rectangle1, Add, FIELD_OF_VIEW_LINE_LAYER, Yellow, 1, 570, 700, 1350, 240);
 }
static uint8_t client_layer0(void)
{
	shoot_line();
	return data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID, (uint8_t *)&judge_send_mesg.layer0_shoot_line,
							sizeof(judge_send_mesg.layer0_shoot_line), DN_REG_ID);
}
static uint8_t client_layer0_1(void)
{

	field_of_view_line();
	return data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID, (uint8_t *)&judge_send_mesg.layer0_field_of_view_line,
							sizeof(judge_send_mesg.layer0_field_of_view_line), DN_REG_ID);
}
static uint8_t client_layer1(double value, char name, char type, char layer, char color, uint32_t start_x, uint32_t start_y)
{
	char string[30] = {0};
	switch (name)
	{
	case Compensates: // P
		strcpy(string, " P : 00.00");
		if (value < 0.0f)
			string[4] = '-';
		break;
	case Yaw:
		strcpy(string, " Y : 00.00");
		if (value < 0.0f)
			string[4] = '-';
		break;
	case Cap:
		strcpy(string, "Cap: 00.0");
		break;
	case Distance:
		strcpy(string, "Dis: 00.0");
		break;
	case Shoot:
		strcpy(string, "Num: 00.0");
		break;
	case Power:
		strcpy(string, "Pow>=00.0");
	default:
		break;
	}
	if (name == Cap)
	{
		if (value < 11.0f)
			value = 0.0f;
		else if (value >= 26.0f)
			value = 99.9f;
		else
			value = (value - 11.0f) * 100.0f / 15.0f;
		if (value >= 60)
			color = Green;
		else
			color = Orange;
	}
	else if (name == Power)
	{
		switch ((uint16_t)value)
		{
		case 0x32:
			value = 50;
			color = Green;
			break;
		case 0x1E:
			value = 30;
			color = Orange;
			break;
		case 0x1C:
			value = 15;
			color = Orange;
			break;
		case 0x18:
			value = 5;
			color = Orange;
		case 0x10:
			value = 1;
			color = Orange;
			break;
		default:
			break;
		}
	}
	else if (name == Power)
	{
		switch ((uint16_t)value)
		{
		case 0x32:
			value = 50;
			color = Green;
			break;
		case 0x1E:
			value = 30;
			color = Orange;
			break;
		case 0x1C:
			value = 15;
			color = Orange;
			break;
		case 0x18:
			value = 5;
			color = Orange;
		case 0x10:
			value = 1;
			color = Orange;
			break;
		default:
			break;
		}
	}

//	if(name == Yaw)
//	{
//		string[5] = (uint8_t)fabs(value) / 100 + '0'; // 48——'0'ASCII码
//		string[6] = (uint8_t)fabs(value) %100 / 10 + '0';
//		string[7] = (uint8_t)fabs(value) % 10 + '0';
//		string[8] = '.';
//		string[9] = (uint8_t)(fmod(fabs(value) * 10.0f, 10.0f) + '0');
//		string[10] = (uint8_t)(fmod(fabs(value) * 100.0f, 10.0f) + '0');
//	}	
//	else
//	{
		string[5] = (uint8_t)fabs(value) / 10 + '0'; // 48——'0'ASCII码
		string[6] = (uint8_t)fabs(value) % 10 + '0';
		string[7] = '.';
		string[8] = (uint8_t)(fmod(fabs(value) * 10.0f, 10.0f) + '0');
		string[9] = (uint8_t)(fmod(fabs(value) * 100.0f, 10.0f) + '0');
//	}
	grapic_struct_assignment_fun(&judge_send_mesg.layer1, Client_Draw_Character_Graph);
	memcpy(&judge_send_mesg.layer1.data[0], string, sizeof(string));
	grapic_string_fun(&judge_send_mesg.layer1.grapic_data_struct, name, type, layer, color, strlen(string), start_x, start_y);
	return data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID, (uint8_t *)&judge_send_mesg.layer1,
							sizeof(judge_send_mesg.layer1), DN_REG_ID);
}
static uint8_t client_layer5(char name, char type, char layer, char color, uint32_t start_x, uint32_t start_y)
{
	char string[30] = {0};
	switch (name)
	{
	case base_15:
		strcpy(string, "15_8.33");
		break;
	case base_17:
		strcpy(string, "17_9.04");
		break;
	default:
		break;
	}
	grapic_struct_assignment_fun(&judge_send_mesg.layer5, Client_Draw_Character_Graph);
	memcpy(&judge_send_mesg.layer5.data[0], string, sizeof(string));
	grapic_string_fun(&judge_send_mesg.layer5.grapic_data_struct, name, type, layer, color, strlen(string), start_x, start_y);
	return data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID, (uint8_t *)&judge_send_mesg.layer5,
							sizeof(judge_send_mesg.layer5), DN_REG_ID);
}
static uint8_t client_layer2(uint32_t operate_tpye)
{
	// 轻触开关
	uint8_t color;
	if (shoot_ready)
		color = Green;
	else
		color = Orange;
	grapic_struct_assignment_fun(&judge_send_mesg.layer2, Client_Draw_Two_Graph);
	grapic_circle_fun(&judge_send_mesg.layer2.interaction_figure[0], circle0, operate_tpye, FIRE_READY_LAYER, color, 2, 952, 540, 400);
	return data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID, (uint8_t *)&judge_send_mesg.layer2,
							sizeof(judge_send_mesg.layer2), DN_REG_ID);
}
static uint8_t client_layer3(uint32_t operate_tpye)
{
	// 摩擦轮
	uint8_t color;
	if (shoot.fric_trun_on)
		color = Green;
	else
		color = Orange;
	grapic_struct_assignment_fun(&judge_send_mesg.layer3, Client_Draw_Two_Graph);
	grapic_circle_fun(&judge_send_mesg.layer3.interaction_figure[0], circle1, operate_tpye, FRIC_WHEEL_LAYER, color, 3, 952, 540, 410);

	if (chassis.dodge_ctrl)
		color = Green;
	else if (chassis.separt_ctrl)
		color = Amaranth;
	else
		color = Orange;

	grapic_line_fun(&judge_send_mesg.layer3.interaction_figure[1], line_4s, operate_tpye, CHASSIS_LAYER, color, 2, 804, 630,
					(804 + 100 * sin(-gimbal.sensor.yaw_relative_angle / 57.0f)),
					(630 + 100 * cos(-gimbal.sensor.yaw_relative_angle / 57.0f))); // 底盘朝向
	return data_packet_pack(STUDENT_INTERACTIVE_HEADER_DATA_ID, (uint8_t *)&judge_send_mesg.layer3,
							sizeof(judge_send_mesg.layer3), DN_REG_ID);
}
static uint8_t value_refalsh(float last_value, float value)
{
	if (value == last_value) // cap
		return 0;
	else
		return 1;
}
uint8_t ui_fun(uint32_t operate_tpye)
{
	if (operate_tpye == Add)
	{
		

		switch (i)
		{
		case 0:
			if (client_layer0())
				i++;
			else
				return 0;
			break;
		case 1:
			if (client_layer0_1())
				i++;
			else
				return 0;
			break;
		case 2:
			if (client_layer1((double)gimbal.sensor.pit_relative_angle, Compensates, operate_tpye, PIT_ANGLE_LAYER, Yellow, 760, 400))
				i++; // pit
			else
			 	return 0;
			break;
		case 3:
			if (client_layer1((double)gimbal.sensor.yaw_relative_angle, Yaw, operate_tpye, YAW_ANGLE_LAYER, Yellow, 760, 350))
				i++; // yaw
			else
			 	return 0;			
			break;
		case 4:
			if (client_layer1((double)(chassis.CapData[1]), Cap, operate_tpye, CAP_LAYER, Yellow, 1100, 570))
				i++; // 电容
			else
				return 0;
			break;
		case 5:
			if (client_layer2(operate_tpye))
				i++; // 轻触开关
			else
				return 0;
			break;
		case 6:
			if (client_layer3(operate_tpye))
				i++;
			else
				return 0;
			break;
		case 7:
			if (client_layer5(base_15, operate_tpye, NUM_TAB, Amaranth, 966, 258))
				i++;
			else
				return 0;
		break;
		case 8:
			if (client_layer5(base_17, operate_tpye, NUM_TAB, Redblue, 966, 224))
				i++;
			else
				return 0;
		break;
//		case 9:
//			if (client_layer5(base_1, operate_tpye, NUM_TAB, Amaranth, 965, 300))
//				i++;
//			else
//				return 0;
//		break;
		default:
			i = 0;
		break;
		}
	}
	else
	{
		switch (i)
		{
		case 0:
			if (value_refalsh(last_value[1], chassis.CapData[1]))
			{
				if (client_layer1((double)(chassis.CapData[1]), Cap, operate_tpye, CAP_LAYER, Yellow, 1100, 570))
					i++; // 电容
				else
					return 0;
			}
			else
				i++;
			break;
		case 1:
			if (value_refalsh(last_value[2], (float)shoot_ready))
			{
				if (client_layer2(operate_tpye))
					i++; // 轻触开关
				else
					return 0;
			}
			else
				i++;
			break;
		case 2:
			if (value_refalsh(last_value[3], gimbal.sensor.yaw_relative_angle))
			{
				if (client_layer3(operate_tpye))
					i++; // 摩擦轮和小陀螺
				else
					return 0;
			}
			else
				i++;
			break;
		case 3:
		 	if (value_refalsh(last_value[0], gimbal.sensor.pit_relative_angle))
		 	{
		 		if (client_layer1((double)gimbal.sensor.pit_relative_angle, Compensates, operate_tpye, PIT_ANGLE_LAYER, Yellow, 760, 400))
		 			i++; // pit
		 		else
		 			return 0;
		 	}
		 	else
		 		i++;
		 	break;
		case 4:
			if (value_refalsh(last_value[4], gimbal.sensor.yaw_relative_angle))
			{
				if (client_layer1((double)gimbal.sensor.yaw_relative_angle, Yaw, operate_tpye, YAW_ANGLE_LAYER, Yellow, 760, 350))
					i++;
				else
					return 0;
			}
			else
				i++;
			break;

		// case 5:
		// 	if (value_refalsh(last_value[6], judge_recv_mesg.buff.remaining_energy))
		// 	{
		// 		if (client_layer1(judge_recv_mesg.buff.remaining_energy, Power, operate_tpye, POWER_LAYER, Yellow, 982, 270))
		// 			i++;
		// 		else
		// 			return 0;
		// 	}
		// 	else
		// 		i++;
		// 	break;
		default:
			i = 0;
			last_value[0] = (double)gimbal.sensor.pit_relative_angle;
			last_value[1] = (double)(chassis.CapData[1]);
			last_value[2] = (double)shoot_ready;
			last_value[3] = (double)gimbal.sensor.yaw_relative_angle;
			last_value[4] = (double)gimbal.sensor.yaw_relative_angle;
			// last_value[6] = (double)judge_recv_mesg.buff.remaining_energy;
			break;
		}
	}
	return 1;
}

uint8_t UI_init_flag = 0;

uint8_t staic_ui_refresh(void)
{
	if (!UI_init_flag)
	{
		// 自定义UI receiver_ID 只能选当前机器人的对应的客户端,解耦化，以后每台车UI都可以用差不多的结构
		current_robot_id = judge_recv_mesg.game_robot_state.robot_id; // 读取当前机器人的id
		receiver_ID = Client_ID_Swich_Fun(current_robot_id);		  // 未用
		last_base_HP = get_base_HP(receiver_ID);
		// last_launchable_shoot_num = judge_recv_mesg.bullet_remaining.projectile_allowance_42mm;
		// last_value[5] = judge_recv_mesg.bullet_remaining.projectile_allowance_42mm;
		//		ui_fun(Add);
		UI_init_flag = 1;
	}
	return ui_fun(Add);
}
uint8_t dynamic_ui_refresh(void)
{
	if (global_mode != RELEASE_CTRL)
	{
		UI_init_flag = 0;
	}
	return ui_fun(Change);
}
