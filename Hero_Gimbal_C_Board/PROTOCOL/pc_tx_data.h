#ifndef _pc_tx_data_H
#define _pc_tx_data_H

#include "stm32f4xx.h"
#include "data_packet.h"

#define PC_TX_FIFO_BUFLEN 500

typedef enum
{
	TRACK_AMOR_MODE			=0,
	BIG_BUFF_MODE				=1,
	NORMAL_CTRL_MODE		=2,
	SMALL_BUFF_MODE			=3,
}mode_t;

typedef enum
{
	RED  = 0,
	BLUE = 1,
}enemy_color_t;

typedef enum
{
	CENTRE  = 0,
	FOLLOW = 1,
}lock_mode_t;

/**
 * @brief  1 + 4*3 = 13字节
 */
#pragma pack(1)
typedef struct
{
  uint8_t robot_color : 1;
  uint8_t visual_valid : 1; 
  int last_shoot_aim_id;
  uint8_t program_mode : 1;
  uint8_t reserved : 1; 
  float robot_roll;
  float robot_pitch;
  float robot_yaw; 
  float bullet_speed;

} robot_tx_data;
#pragma pack()


//#pragma pack(1)
//typedef struct
//{ 
//  uint8_t robot_color : 1;			//敌方颜色
//  uint8_t visual_valid : 1; 		//自瞄有效位，判断是否识别到装甲板
//	int last_shoot_aim_id;				//上一次识别的装甲ID，此数据来自算法
//	uint8_t program_mode : 1;			//是否进入自瞄
//  uint8_t reserved : 1; 				//保留位，保证通信
////  uint8_t reserved : 6; 				//保留位，保证通信
//	
//	float robot_roll;							//陀螺仪三轴
//  float robot_pitch;
//  float robot_yaw; 
//	
//  float bullet_speed;						//当前弹速
//} robot_tx_data;
//#pragma pack()


/*

uint8_t robot_color : 1;  // 敌方颜色
  uint8_t task_mode : 3;    // 当前任务模式
  uint8_t visual_valid : 1; // 视觉有效位
 // uint8_t direction : 2;    // 扩展装甲板方向
  uint8_t bullet_level : 1; // 弹速
  float robot_pitch;        // 当前pitch位置
  float robot_yaw;          // 当前yaw位置 
  float time_stamp;         // 时间
*/



extern fifo_s_t  pc_txdata_fifo;
extern robot_tx_data pc_send_mesg;

void pc_tx_param_init(void);
void pc_send_data_packet_pack(void);
void get_upload_data(void);
void get_infantry_info(void);


#endif
