#ifndef _judge_rx_data_H
#define _judge_rx_data_H

#include "stm32f4xx.h"
#include "data_packet.h"

#define JUDGE_RX_FIFO_BUFLEN 256

/**代码相对于21赛季裁判系统通信缺少模块如下***

*0x0004 -- 飞镖发射状态，飞镖发射时发送     已添加
*0x0005 -- 人工智能挑战赛加成与惩罚区状态   已添加

*0x0103 -- 请求补给站补弹数据（由参赛队发送 上限10HZ RM对抗赛尚未开发）未添加
*0x0104 -- 裁判警告数据，警告发生后发送     已添加
*0x0105 -- 飞镖发射口倒计时                已添加

*0x0208 -- 弹丸剩余发射数，仅空中机器人、哨兵机器人以及ICRA机器人发送 已添加
*0x0209 -- 机器人RFID状态                  已添加
*0x020A -- 飞镖机器人客户端指令数据         已添加
**/
typedef enum
{
	GAME_STATE_ID = 0x0001,							// 比赛状态数据
	GAME_RESULT_ID = 0x0002,						// 比赛结果数据
	GAME_ROBOT_HP_ID = 0x0003,						// 比赛机器人血量数据
	EVENT_DATA_ID = 0x0101,							// 场地事件数据
	REFEREE_WARNING_ID = 0x0104,					// 裁判系统警告信息
	DART_REMAINING_TINME_ID = 0x0105,				// 飞镖发射相关数据
	GAME_ROBOT_STATE_ID = 0x0201,					// 机器人性能体系数据
	POWER_HEAT_DATA_ID = 0x0202,					// 实时底盘缓冲能量和射击热量数
	GAME_ROBOT_POS_ID = 0x0203,						// 机器人位置数据
	BUFF_ID = 0x0204,								// 机器人增益数据
	ROBOT_HURT_ID = 0x0206,							// 伤害状态数据，伤害发生后发送
	SHOOT_DATA_ID = 0x0207,							// 实时射击数据，子弹发射后发送
	BULLET_REMAINING_ID = 0x0208,					// 允许发弹量
	RFID_STATE_ID = 0x0209,							// 机器人RFID模块状态
	DAT_CLIENT_CMD_ID = 0x020A,						// 飞镖机器人客户端指令数据
	GROUND_ROBOT_POSITION = 0x020B,					// 地面机器人位置数据
	RADAR_MARK_ROBOT_PROGRESS = 0x020C,				// 雷达标记进度数据
	SENTRY_ROBOT_INFO = 0x020D,						// 哨兵自主决策信息同步
	RADAR_MARK_INFO = 0x020E,						// 雷达自主决策信息同步
	STUDENT_INTERACTIVE_HEADER_DATA_ID = 0x0301,	// 机器人间交互数据 和 客户端通信（发送方触发发送）
	CUSTOM_CONTROLLER_INTERACTION_DATA_ID = 0x0302, // 自定义控制器交互数据接口，通过客户端触发发送，上限 30HZ
	CLIENT_MINIMAP_INTERACTIVE_DATDA_ID = 0x0303,	// 选手端小地图交互数据
	KEYBOARD_AND_MOUSE_INFORMATION_ID = 0x0304,		// 键盘、鼠标信息，通过图传串口发送
	PLAYER_RECEIVE_RADAR_DATA = 0x0305,				// 选手端小地图接收雷达数据
	CONTROLLER_DATA_WITH_PLAYER = 0x0306,			// 自定义控制器与选手端交互数据
	PLAYER_RECEIVE_SENTRY_DATA = 0x0307,			// 选手端小地图接收哨兵数据
	PLAYER_RECEIVE_ROBOT_DATA = 0x0308,				// 选手端小地图接收机器人数据
	COTROLLER_RECEIVE_ROBOT_DATA = 0x0309,			// 自定义控制器接收机器人数据

} judge_data_id_e;

/*-------------------------------------------------------------*/

/*0001（2025）*/
typedef __packed struct
{
	uint8_t game_type : 4;
	uint8_t game_progress : 4;
	uint16_t stage_remain_time;
	uint64_t SyncTimeStamp;
} game_status_t;

/*-------------------------------------------------------------*/

/*0002（2025）*/
typedef __packed struct
{
	uint8_t winner;

} game_result_t;

/*-------------------------------------------------------------*/

/*0003（2025）*/
typedef __packed struct
{
	uint16_t red_1_robot_HP;
	uint16_t red_2_robot_HP;
	uint16_t red_3_robot_HP;
	uint16_t red_4_robot_HP;
	uint16_t reserved;
	uint16_t red_7_robot_HP;
	uint16_t red_outpost_HP;
	uint16_t red_base_HP;
	uint16_t blue_1_robot_HP;
	uint16_t blue_2_robot_HP;
	uint16_t blue_3_robot_HP;
	uint16_t blue_4_robot_HP;
	uint16_t reserveds;
	uint16_t blue_7_robot_HP;
	uint16_t blue_outpost_HP;
	uint16_t blue_base_HP;
} game_robot_HP_t;

/*0101（2025）*/
typedef __packed struct
{
	uint32_t event_data;
} event_data_t;

/*-------------------------------------------------------------*/

/*0104（2025）*/
typedef __packed struct
{
	uint8_t level;
	uint8_t offending_robot_id;
	uint8_t count;
} referee_warning_t;

/*-------------------------------------------------------------*/

/*0105（2025）*/
typedef __packed struct
{
	uint8_t dart_remaining_time;
	uint16_t dart_info;
} dart_info_t;

/*-------------------------------------------------------------*/

/*0201（2025）*/
typedef __packed struct
{
	uint8_t robot_id;
	uint8_t robot_level;
	uint16_t current_HP;
	uint16_t maximum_HP;
	uint16_t shooter_barrel_cooling_value;
	uint16_t shooter_barrel_heat_limit;
	uint16_t chassis_power_limit;
	uint8_t power_management_gimbal_output : 1;
	uint8_t power_management_chassis_output : 1;
	uint8_t power_management_shooter_output : 1;
} robot_status_t;

/*-------------------------------------------------------------*/

/*0202（2025）*/
typedef __packed struct
{
	uint16_t reserved;
	uint16_t reserved1;
	float reserved2;
	uint16_t buffer_energy;
	uint16_t shooter_17mm_1_barrel_heat;
	uint16_t shooter_17mm_2_barrel_heat;
	uint16_t shooter_42mm_barrel_heat;
} power_heat_data_t;

/*-------------------------------------------------------------*/

/*0203（2025）*/
typedef __packed struct
{
	float x;
	float y;
	float angle;
} robot_pos_t;

/*-------------------------------------------------------------*/

/*0204（2025）*/
typedef __packed struct
{
	uint8_t recovery_buff;
	uint8_t cooling_buff;
	uint8_t defence_buff;
	uint8_t vulnerability_buff;
	uint16_t attack_buff;
	uint8_t remaining_energy;
} buff_t;

/*-------------------------------------------------------------*/

/*0206（2025）*/
typedef __packed struct
{
	uint8_t armor_id : 4;
	uint8_t HP_deduction_reason : 4;
} hurt_data_t;

/*-------------------------------------------------------------*/

/*0207（2025）*/
typedef __packed struct
{
	uint8_t bullet_type;
	uint8_t shooter_number;
	uint8_t launching_frequency;
	float initial_speed;
} shoot_data_t;

/*-------------------------------------------------------------*/

/*0208（2025）*/
typedef __packed struct
{
	uint16_t projectile_allowance_17mm;
	uint16_t projectile_allowance_42mm;
	uint16_t remaining_gold_coin;
} projectile_allowance_t;

/*-------------------------------------------------------------*/

/*0209（2024）*/
typedef __packed struct
{
	uint32_t rfid_status;
} rfid_status_t;

/*-------------------------------------------------------------*/

/*020A（2025）*/
typedef __packed struct
{
	uint8_t dart_launch_opening_status;
	uint8_t reserved;
	uint16_t target_change_time;
	uint16_t latest_launch_cmd_time;
} dart_client_cmd_t;

/*-------------------------------------------------------------*/
/*020B（2025）新增*/
typedef __packed struct
{
	float hero_x;
	float hero_y;
	float engineer_x;
	float engineer_y;
	float standard_3_x;
	float standard_3_y;
	float standard_4_x;
	float standard_4_y;
	float reserved;
	float reserved1;
} ground_robot_position_t;

/*-------------------------------------------------------------*/

/*020C（2025）新增*/
typedef __packed struct
{
	uint8_t mark_progress;
} radar_mark_data_t;

/*-------------------------------------------------------------*/

/*020D（2025）新增*/
typedef __packed struct
{
	uint32_t sentry_info;
	uint16_t sentry_info_2;
} sentry_info_t;

/*-------------------------------------------------------------*/

/*020E（2025）新增*/
typedef __packed struct
{
	uint8_t radar_info;
} radar_info_t;

/*-------------------------------------------------------------*/
/*0x0301*/ // tx.h里面

/*-------------------------------------------------------------*/
///*0304（2025）*/
// typedef _packed struct
//{
//	int16_t mouse_x;
//	int16_t mouse_y;
//	int16_t mouse_z;
//	int8 left_button_down;
//	int8 right_button_down;
//	uint16_t keyboard_value;
//	uint16_t reserved;
// }remote_control_t;

/*-------------------------------------------------------------*/

// /*0303*/
// typedef __packed struct
// {
// 	float target_position_x;
// 	float target_position_y;
// 	float target_position_z;
// 	uint8_t commd_keyboard;
// 	uint16_t target_robot_ID;
// }ext_robot_command1_t;
// /*0304*/
// typedef __packed struct
// {
// 	int16_t mouse_x;
// 	int16_t mouse_y;
// 	int16_t mouse_z;
// 	int8_t left_button_down;
// 	int8_t right_button_down;
// 	uint16_t keyboard_value;
// 	uint16_t reserved;
// }ext_robot_command2_t;

typedef struct
{
	game_status_t game_state;	   // 0x0001
	game_result_t game_result;	   // 0x0002
	game_robot_HP_t game_robot_HP; // 0x0003
	//	ext_dart_status_t                   dart_state;                  		 //0x0004 //(2023/2024表均无但还是写上去了？)
	//	ext_ICRA_buff_debuff_zone_status_   ICRA_buff_debuff_zone_state; 		 //0x0005 //(2023/2024表均无但还是写上去了？)
	event_data_t event_data; // 0x0101
	//	ext_supply_projectile_action_t   	supply_projectile_action;  	 		 //0x0102
	//  ext_supply_projectile_booking_t     supply_projectile_booking;   		 //0x0103
	referee_warning_t referee_warning; // 0x0104
	dart_info_t dart_remaining_time;   // 0x0105
	robot_status_t game_robot_state;   // 0x0201
	power_heat_data_t power_heat_data; // 0x0202
	robot_pos_t game_robot_pos;		   // 0x0203
	buff_t buff;					   // 0x0204
	//	air_support_data_t				 	aerial_robot_energy;		 		 //0x0205
	hurt_data_t robot_hurt;					 // 0x0206
	shoot_data_t shoot_data;				 // 0x0207
	projectile_allowance_t bullet_remaining; // 0x0208
	rfid_status_t rfid_state;				 // 0x0209
	dart_client_cmd_t dart_client_cmd;		 // 0x020A

	ground_robot_position_t ground_robot_pos; // 0x020B 2024新增
	radar_mark_data_t radar_mark_data;		  // 0x020C 2024新增
	sentry_info_t sentry_info;				  // 0x020D 2024新增
	radar_info_t radar_info;				  // 0x020E 2024新增

	// robot_interactive_data_t           robot_interactive_data;  //0x0302
	// ext_robot_command1_t               minimap_interactive_data;//0x0303
	// ext_robot_command2_t               mouse_keyboard_informationt;//
} judge_rxdata_t;

extern judge_rxdata_t judge_recv_mesg; // 读取回来保存在该变量
extern uart_dma_rxdata_t judge_rx_obj;
extern unpack_data_t judge_unpack_obj;

void judgement_rx_param_init(void);
void judgement_data_handler(uint8_t *p_frame);

#endif
