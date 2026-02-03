#ifndef _judge_tx_data_H
#define _judge_tx_data_H

#include "stm32f4xx.h"
#include "data_packet.h"

#define JUDGE_TX_FIFO_BUFLEN JUDGE_TX_LEN					 // 用于给发送邮箱的初始化 500
#define STUDENT_DATA_LENGTH sizeof(robot_interaction_data_t) // 交互数据的长度

#define SHOOT_LINE_LAYER layer0
#define FIELD_OF_VIEW_LINE_LAYER layer0
#define CAP_LAYER layer1
#define PIT_ANGLE_LAYER layer1
#define YAW_ANGLE_LAYER layer1
#define SHOOT_NUM_LAYER layer1
#define POWER_LAYER layer1
#define NUM_TAB	layer5
#define FIRE_READY_LAYER layer2
#define CHASSIS_LAYER layer3
#define FRIC_WHEEL_LAYER layer3
#define DYNAMIC_SHOOT_LINE_LAYER layer4

typedef enum
{
	dog_chassis_line_angle = 90,
	dog_chassis_circle0_angle = 45,
	dog_chassis_circle1_angle = 135,
	dog_chassis_circle2_angle = 225,
	dog_chassis_circle3_angle = 315,
} dog_angle;

/*0x0301 机器人间交互数据 和 客户端通信*/
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[112]; // 最大为112字节
} robot_interaction_data_t;
// 学生交互id
typedef enum
{
	// 学生交互机器人id
	STUDENT_RED_HERO_ID = 1,
	STUDENT_RED_ENGINEER_ID = 2,
	STUDENT_RED_INFANTRY3_ID = 3,
	STUDENT_RED_INFANTRY4_ID = 4,
	STUDENT_RED_INFANTRY5_ID = 5,
	STUDENT_RED_AERIAL_ID = 6,
	STUDENT_RED_SENTRY_ID = 7,
	STUDENT_RED_RadarStation_ID = 9,
	STUDENT_RED_Outpost_ID = 10, // 前哨站(前哨和基地的ID用于小地图交互数据)
	STUDENT_RED_Base_ID = 11,	 // 基地
	STUDENT_BLUE_HERO_ID = 101,
	STUDENT_BLUE_ENGINEER_ID = 102,
	STUDENT_BLUE_INFANTRY3_ID = 103,
	STUDENT_BLUE_INFANTRY4_ID = 104,
	STUDENT_BLUE_INFANTRY5_ID = 105,
	STUDENT_BLUE_AERIAL_ID = 106,
	STUDENT_BLUE_SENTRY_ID = 107,
	STUDENT_BLUE_RadarStation_ID = 109,
	STUDENT_BLUE_Outpost_ID = 110,
	STUDENT_BLUE_Base_ID = 111,

	// 机器人对应客户端id
	RED_HERO_CLIENT_ID = 0x0101,
	RED_ENGINEER_CLIENT_ID = 0x0102,
	RED_INFANTRY3_CLIENT_ID = 0x0103,
	RED_INFANTRY4_CLIENT_ID = 0x0104,
	RED_INFANTRY5_CLIENT_ID = 0x0105,
	RED_AERIAL_CLIENT_ID = 0x0106,
	BLUE_HERO_CLIENT_ID = 0x0165,
	BLUE_ENGINEER_CLIENT_ID = 0x0166,
	BLUE_INFANTRY3_CLIENT_ID = 0x0167,
	BLUE_INFANTRY4_CLIENT_ID = 0x0168,
	BLUE_INFANTRY5_CLIENT_ID = 0x0169,
	BLUE_AERIAL_CLIENT_ID = 0x016A,
} interactive_id_e;

// 内容ID和命令ID
typedef enum
{
	/*stm32是小端模式，比如一个32位无符号数0x12345678,
	从低地址到高地址依次存储78、56、34、12

	32bit宽的数0x12345678在小端模式及大端模式CPU中存放的方式（假设地址从0x4000开始存放）为：
	内存地址		小端模式存放内容		大端模式存放内容
	0x4000			0x78								0x12
	0x4001			0x56								0x34
	0x4002			0x34								0x56
	0x4003			0x12								0x78							*/

	// 内容ID
	RobotCommunication = 0x0200, // 可以在0x0200~0x02FF中自行添加一个或多个作为交互ID

	Client_Delete_Graph = 0x0100,		  // 选手端删除图层
	Client_Draw_One_Graph = 0x0101,		  // 选手端绘制一个图形
	Client_Draw_Two_Graph = 0x0102,		  // 选手端绘制两个图形
	Client_Draw_Five_Graph = 0x0103,	  // 选手端绘制五个图形
	Client_Draw_Seven_Graph = 0x0104,	  // 选手端绘制七个图形
	Client_Draw_Character_Graph = 0x0110, // 选手端绘制字符图形

} Content_ID;

// 0x0100 客户端删除图形 2025
typedef __packed struct
{
	uint8_t delete_type;
	uint8_t layer;
} interaction_layer_delete_t;

// 0x0101 图形数据配置，具体查看裁判系统串口协议附录 2025
typedef __packed struct
{
	uint8_t figure_name[3];
	uint32_t operate_tpye : 3; // 类型说明符 位域名：位域长度,后面的位域长度表示该位域占了多少个位
	uint32_t figure_tpye : 3;
	uint32_t layer : 4;
	uint32_t color : 4;

	uint32_t details_a : 9;
	uint32_t details_b : 9;
	uint32_t width : 10;
	uint32_t start_x : 11;
	uint32_t start_y : 11;
	uint32_t details_c : 10;
	uint32_t details_d : 11;
	uint32_t details_e : 11;
} interaction_figure_t; // interaction_figure_t

// 0x0102 绘制 图形1、图形2 2025
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	interaction_figure_t interaction_figure[2];
} interaction_figure_2_t;

// 0x0103 绘制 图形1、图形2、图形3、图形4、图形5  2025
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	interaction_figure_t interaction_figure[5];
} interaction_figure_3_t;

// 0x0104 绘制 图形1、图形2、图形3、图形4、图形5、图形6、图形7  2025
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	interaction_figure_t interaction_figure[7];
} interaction_figure_4_t;

// 0x0110 绘制（字符串）cap 2025
typedef __packed struct
{
	uint16_t data_cmd_id;
	uint16_t sender_ID;
	uint16_t receiver_ID;
	interaction_figure_t grapic_data_struct; // 2024 v1.6.1 手册可能写错了 ？写成了graphic_data_struct_t ？但是他们位数是一样的,不用修改
	uint8_t data[30];
} ext_client_custom_character_t;

// 0x0120 哨兵自主决策指令 2025
typedef __packed struct
{
	uint32_t sentry_cmd;
} sentry_cmd_t;

// 0x0121 雷达自主决策指令2025
typedef __packed struct
{
	uint8_t radar_cmd;
} radar_cmd_t;

// 0x0303 小地图交互数据 2025
typedef __packed struct
{
	float target_position_x;
	float target_position_y;
	uint8_t cmd_keyboard;
	uint8_t target_robot_id;
	uint8_t cmd_source;
} map_command_t;

// 选手端小地图可接收机器人数据,雷达可通过常规链路向己方所有选手端发送对方机器人的坐标数据，该位置会在己方选手端小地图显示 2024
typedef __packed struct
{
	uint16_t target_robot_id;
	float target_position_x;
	float target_position_y;
} map_robot_data_t;

// 0x0307 小地图坐标 2024
typedef __packed struct
{
	uint8_t intention;
	uint16_t start_position_x;
	uint16_t start_position_y;
	int8_t delta_x[49];
	int8_t delta_y[49];
	uint16_t sender_id;
} map_data_t;
// 己方机器人可通过常规链路向己方任意选手端发送自定义的消息，该消息会在己方选手端特定位置显示 2024
typedef __packed struct
{
	uint16_t sender_id;
	uint16_t receiver_id;
	uint8_t user_data[30];
} custom_info_t;

// 自定义控制器与机器人交互数据 2024
typedef __packed struct
{
	uint8_t data[30];
} custom_robot_data_t;
// 通过遥控器发送的键鼠遥控数据将同步通过图传链路发送给对应机器人 2024
typedef __packed struct
{
	int16_t mouse_x;
	int16_t mouse_y;
	int16_t mouse_z;
	int8_t left_button_down;
	int8_t right_button_down;
	uint16_t keyboard_value;
	uint16_t reserved;
} remote_control_t;
// 操作手可使用自定义控制器模拟键鼠操作选手端 2024
typedef __packed struct
{
	uint16_t key_value;
	uint16_t x_position : 12;
	uint16_t mouse_left : 4;
	uint16_t y_position : 12;
	uint16_t mouse_right : 4;
	uint16_t reserved;
} custom_client_data_t;

// 图形设置
typedef enum
{
	// 图形命名，可自行添加
	Fric = 1,
	DODGE = 2,
	SENTRY_line = 3,
	Pitch = 4,
	line_1s = 5,
	line_2s = 6,
	line_3s = 7,
	line_4s = 8,
	line_5s = 9,
	line_6s = 10,
	line_7s = 11,
	line_8s = 12,
	line_9s = 13,
	line_10s = 14,
	line_11s = 15,
	line_12s = 16,
	line_13s = 17,
	line_14s = 18,
	line_15s = 19,
	line_16l = 20,
	line_17l = 21,
	line_18l = 22,
	line_19l = 23,
	line_20l = 24,
	line_1v = 25,
	line_1c = 26,
	line_2c = 27,
	line_3c = 28,
	line_4c = 29,
	line_5c = 30,
	Text = 31,
	Chassis = 32,
	Gimbal = 33,
	Shoot = 34,
	Cap = 35,
	View = 36,
	circle0 = 37,
	circle1,
	circle2,
	circle3,
	circle4,
	circle5,
	circle6,
	circle7,
	rectangle0,
	rectangle1,
	rectangle2,
	rectangle3,
	rectangle4,
	Distance,
	Compensates,
	Yaw,
	Speed,
	Vision,
	Rectangle1,
	Power,
	base_15,
	base_15_5,
	base_16,
	base_17,
	

	// 图形操作
	Null_operate = 0, // 空操作
	Delete_graph = 1, // 删除图层？
	Delete_all = 2,	  // 删除所有 ？

	Add = 1,	// 增加
	Change = 2, // 修改
	Delete = 3, // 删除

	// 图形类型
	Straight_line = 0, // 直线
	Rectangle = 1,	   // 矩形
	Circle = 2,		   // 正圆
	ellipse = 3,	   // 椭圆
	Circular_arc = 4,  // 圆弧
	Floatnumber = 5,   // 浮点数
	Intnumber = 6,	   // 整形数
	Character = 7,	   // 字符

	// 图层数
	layer0 = 0,
	layer1 = 1,
	layer2 = 2,
	layer3 = 3,
	layer4 = 4,
	layer5 = 5,
	layer6 = 6,
	layer7 = 7,
	layer8 = 8,
	layer9 = 9,

	// 颜色
	Redblue = 0,  // 红/蓝(己方颜色)
	Yellow = 1,	  // 黄色
	Green = 2,	  // 绿色
	Orange = 3,	  // 橙色
	Amaranth = 4, // 紫红色
	Pink = 5,	  // 粉色
	Cyan = 6,	  // 青色
	Black = 7,	  // 黑色
	White = 8,	  // 白色
} client_graphic_draw_operate_data_e;

/**
 * @brief  the data structure receive from judgement
 */
typedef struct
{
	// 0x0200~0x02FF发送的内容
	robot_interaction_data_t student_interactive_header_data;
	// 0x0100~0x0110发送的内容
	interaction_figure_4_t layer0_shoot_line;		  // 射击线
	interaction_figure_2_t layer0_field_of_view_line; // 视觉识别范围

	ext_client_custom_character_t layer1; // 图层1 电容、pit角度
	interaction_figure_2_t layer2;		  // 轻触开关
	interaction_figure_3_t layer3;		  // 底盘状态、摩擦轮状态
	interaction_figure_3_t layer4;		  // 动态瞄准线
	ext_client_custom_character_t layer5;
										  // // 字符绘制
										  // ext_client_custom_character_t ext_client_custom_character;		   // 超电视觉（字符串）
										  // ext_client_custom_character_t ext_client_custom_character_chassis; // 底盘状态（字符串）
										  // ext_client_custom_character_t ext_client_custom_character_shoot;   // 射击状态（字符串）
										  // ext_client_custom_character_t ext_client_custom_character_gimbal;  // 云台状态（字符串）

} judge_txdata_t;

/*debug结构体*/

#define debug 1

typedef struct
{
	uint32_t debug_start_angle;
	uint32_t debug_end_angle;
	uint32_t debug_width;
	uint32_t debug_start_x;
	uint32_t debug_start_y;
	uint32_t debug_radius;
	uint32_t debug_end_x;
	uint32_t debug_end_y;
} judge_debug_t;

extern judge_txdata_t judge_send_mesg;
extern fifo_s_t judge_txdata_fifo;

void judgement_tx_param_init(void);
void judgement_client_packet_pack(uint8_t *p_data);
uint8_t staic_ui_refresh(void);
uint8_t dynamic_ui_refresh(void);
extern double last_value[7];

#endif
