#include "imu_task.h"
#include "gimbal_task.h"
#include "pid.h"
#include "sys_config.h"
#include "math.h"
#include "FreeRTOSConfig.h"
#include "FreeRTOS.h"
#include "task.h"
#include "bsp_can.h"
#include "comm_task.h"
#include "bmi088driver.h"
#include "delay.h"
#include "MahonyAHRS.h"
#include "filters.h"
#include "bsp_dwt.h"
#include "dma.h"
#include "detect_task.h"
#include "ahrs.h"
//debug
#include "usart.h"
#include "modeswitch_task.h"
#ifndef RAD_TO_ANGLE
#define RAD_TO_ANGLE 57.295779513082320876798154814105f
#endif

UBaseType_t imu_stack_surplus;
extern TaskHandle_t imu_Task_Handle;


volatile uint8_t imu_start_flag = 0;

extern IMU_Data_t BMI088;
imu_attitude_t atti;

uint32_t time=0;
float dt=0;
float EulerAngle[3]={0};

float totalangle_transfer(float angle);

float imu_pid[3] = {200,0.1,0};


fp32 gyro_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 gyro_offset[3];

fp32 gyro_cali_offset[3];

fp32 accel_scale_factor[3][3] = {BMI088_BOARD_INSTALL_SPIN_MATRIX};
fp32 accel_offset[3];
fp32 accel_cali_offset[3];

//ist8310_real_data_t ist8310_real_data;
fp32 mag_scale_factor[3][3] = {IST8310_BOARD_INSTALL_SPIN_MATRIX};
fp32 mag_offset[3];
fp32 mag_cali_offset[3];

//static uint8_t first_temperate;
//static const fp32 imu_temp_PID[3] = {TEMPERATURE_PID_KP, TEMPERATURE_PID_KI, TEMPERATURE_PID_KD};
//static pid_type_def imu_temp_pid;

//static const float timing_time = 0.001f;   //tast run time , unit s.任务运行的时间 单位 s

static fp32 accel_fliter_1[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_2[3] = {0.0f, 0.0f, 0.0f};
static fp32 accel_fliter_3[3] = {0.0f, 0.0f, 0.0f};
static const fp32 fliter_num[3] = {1.929454039488895f, -0.93178349823448126f, 0.002329458745586203f};




static fp32 INS_gyro[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_accel[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_mag[3] = {0.0f, 0.0f, 0.0f};
static fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
static fp32 gyro_filter[3] = {0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //euler angle, unit rad.??? ?? rad
fp32 INS_angle_final[3] = {0.0f, 0.0f, 0.0f}; // 转换单位后的角度
fp32 INS_palstance_final[3] = {0.0f, 0.0f, 0.0f}; // 角速度
uint8_t INS_Init_Done = 0;

void imu_temp_keep(void)
{
  PID_Struct_Init(&pid_imu_tmp, imu_pid[0],imu_pid[1],imu_pid[2],5000, 1000, INIT );
	if(BMI088.Temperature>45)
	{
		pid_calc(&pid_imu_tmp, BMI088.Temperature, 45);
		TIM_SetCompare1(TIM10,pid_imu_tmp.out);
	}
	else
	{
		TIM_SetCompare1(TIM10,5000-1); 
  }
}

/**
  * @brief          旋转陀螺仪,加速度计和磁力计,并计算零漂,因为设备有不同安装方式
  * @param[out]     gyro: 加上零漂和旋转
  * @param[out]     accel: 加上零漂和旋转
  * @param[in]      bmi088: 陀螺仪和加速度计数据
  * @retval         none
  */
static void imu_cali_slove(fp32 gyro[3], fp32 accel[3], fp32 mag[3], IMU_Data_t *bmi088)
{
	for (uint8_t i = 0; i < 3; i++)
	{
	
		gyro[i] = (bmi088->Gyro[0] - BMI088.GyroOffset[0]) * gyro_scale_factor[i][0]
						+ (bmi088->Gyro[1] - BMI088.GyroOffset[1]) * gyro_scale_factor[i][1]
						+ (bmi088->Gyro[2] - BMI088.GyroOffset[2]) * gyro_scale_factor[i][2];
		accel[i] 	= bmi088->Accel[0] * accel_scale_factor[i][0]
							+ bmi088->Accel[1] * accel_scale_factor[i][1]
							+ bmi088->Accel[2] * accel_scale_factor[i][2];
//		mag[i] = ist8310->Mag[0] * mag_scale_factor[i][0] + ist8310->Mag[1] * mag_scale_factor[i][1] + ist8310->Mag[2] * mag_scale_factor[i][2] + mag_offset[i];
	}
}

void imu_task(void const *argu)
{
  	u16 i_init = 0;
	uint32_t time_last = 0;
	uint32_t imu_wake_time = osKernelSysTick();

	imu_Task_Handle = xTaskGetHandle(pcTaskGetName(NULL));
	imu_start_flag = 1;
	float yaw_angle_last = 0;


	AHRS_init(INS_quat, INS_accel, INS_mag);
	accel_fliter_1[0] = accel_fliter_2[0] = accel_fliter_3[0] = INS_accel[0];
	accel_fliter_1[1] = accel_fliter_2[1] = accel_fliter_3[1] = INS_accel[1];
	accel_fliter_1[2] = accel_fliter_2[2] = accel_fliter_3[2] = INS_accel[2];
	
  while(1)
  {
		if(i_init < 1000)
			i_init++;
		else
			INS_Init_Done = 1;
//		imu_temp_keep();//IMU恒温处理
    	BMI088_read(BMI088.Gyro, BMI088.Accel, &BMI088.Temperature);
			//	printf("%lf\n",BMI088.Gyro[0]);
		
		//rotate and zero drift 
		imu_cali_slove(INS_gyro, INS_accel, INS_mag, &BMI088);
		
		//accel low-pass filter
		accel_fliter_1[0] = accel_fliter_2[0];
		accel_fliter_2[0] = accel_fliter_3[0];

		accel_fliter_3[0] = accel_fliter_2[0] * fliter_num[0] + accel_fliter_1[0] * fliter_num[1] + INS_accel[0] * fliter_num[2];

		accel_fliter_1[1] = accel_fliter_2[1];
		accel_fliter_2[1] = accel_fliter_3[1];

		accel_fliter_3[1] = accel_fliter_2[1] * fliter_num[0] + accel_fliter_1[1] * fliter_num[1] + INS_accel[1] * fliter_num[2];

		accel_fliter_1[2] = accel_fliter_2[2];
		accel_fliter_2[2] = accel_fliter_3[2];

		accel_fliter_3[2] = accel_fliter_2[2] * fliter_num[0] + accel_fliter_1[2] * fliter_num[1] + INS_accel[2] * fliter_num[2];
#if (ORDER>0)
			//陀螺仪低通滤波
			if(i_filter<ORDER+1)
			{

				i_filter++;
				Filter(INS_gyro[0],gyro_filter_input[0],WINDOWS);
				Filter(INS_gyro[1],gyro_filter_input[1],WINDOWS);
				Filter(INS_gyro[2],gyro_filter_input[2],WINDOWS);
				gyro_filter[0] = INS_gyro[0];
				gyro_filter[1] = INS_gyro[1];
				gyro_filter[2] = INS_gyro[2];
			}
			else
			{
				gyro_filter[0] = Filter(INS_gyro[0],gyro_filter_input[0],WINDOWS);
				gyro_filter[1] = Filter(INS_gyro[1],gyro_filter_input[1],WINDOWS);
				gyro_filter[2] = Filter(INS_gyro[2],gyro_filter_input[2],WINDOWS);
			}
		#else
				gyro_filter[C_BOARD_LENGTH] = INS_gyro[0];
				gyro_filter[C_BOARD_WIDTH]	= INS_gyro[1];
				gyro_filter[C_BOARD_HEIGHT] = INS_gyro[2];
		#endif
		dt = DWT_GetDeltaT(&time_last);
		AHRS_update(INS_quat, dt, INS_gyro, accel_fliter_3, INS_mag);
		INS_angle[C_BOARD_HEIGHT] = get_yaw(INS_quat);
		INS_angle[C_BOARD_WIDTH]	= get_pitch(INS_quat);
		INS_angle[C_BOARD_LENGTH]	= get_roll(INS_quat);	
			
//		get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
//		
//		INS_angle[0]=INS_angle[0]*RAD_TO_ANGLE;		
//		INS_angle[1]=INS_angle[1]*RAD_TO_ANGLE;
//    INS_angle[2]=INS_angle[2]*RAD_TO_ANGLE;
		/**********将三轴陀螺仪弧度转为角度,并且C板安装不同对应的转轴不同，只需在此处调换  PIT  YAW  ROLL  **********/
				INS_angle_final[IMU_ROL] = INS_angle[C_BOARD_LENGTH] * RAD_TO_ANGLE;		//C板的长为轴，灯一侧抬高为正
		INS_palstance_final[IMU_ROL] = gyro_filter[C_BOARD_LENGTH];

				INS_angle_final[IMU_YAW] = INS_angle[C_BOARD_HEIGHT] * RAD_TO_ANGLE;		//C板的高为轴，C板朝上逆时针为正
		INS_palstance_final[IMU_YAW] = gyro_filter[C_BOARD_HEIGHT];

				INS_angle_final[IMU_PIT] = -INS_angle[C_BOARD_WIDTH] * RAD_TO_ANGLE;		//C板的宽为轴，灯一侧抬高为正
		INS_palstance_final[IMU_PIT] = -gyro_filter[C_BOARD_WIDTH];
/***************************************************************************************************88****/
	// 方向     正负
	// pitch	  上正下负
	// yaw			左正右负
  // roll			左正右负
		//云台欧拉角与角速度获取
		gimbal.sensor.yaw_gyro_angle	= INS_angle_final[IMU_YAW];			//陀螺仪角度
		gimbal.sensor.pit_gyro_angle	= INS_angle_final[IMU_PIT];			//陀螺仪角度	
		gimbal.sensor.roll_gyro_angle = INS_angle_final[IMU_ROL];			//陀螺仪角度	
		gimbal.sensor.yaw_palstance = INS_palstance_final[IMU_YAW];		//角速度
		gimbal.sensor.pit_palstance = INS_palstance_final[IMU_PIT];		//角速度
			
		
//		if(gimbal_mode != GIMBAL_RELEASE)		
		
		if(1)
		{
			if(gimbal.sensor.yaw_gyro_angle - yaw_angle_last < -180)
			{
				gimbal.sensor.yaw_cnt++;
			}
			else if(gimbal.sensor.yaw_gyro_angle - yaw_angle_last > 180)
			{
				gimbal.sensor.yaw_cnt--;
			}
//			gimbal.sensor.yaw_total_angle = 360.0f*gimbal.sensor.yaw_cnt + gimbal.sensor.yaw_gyro_angle;
		}

		
		yaw_angle_last = gimbal.sensor.yaw_gyro_angle;
		
		imu_stack_surplus = uxTaskGetStackHighWaterMark(NULL);
		vTaskDelayUntil(&imu_wake_time, IMU_TASK_PERIOD);
		err_detector_hook(IMU_OFFLINE);//看门狗

  }

}

/* Transform into continuous values */
float totalangle_transfer(float angle)
{
  if (angle - atti.last_yaw > 330)
    atti.yaw_cnt--;
  else if (angle - atti.last_yaw < -330)
    atti.yaw_cnt++;
  
  atti.last_yaw = angle;
  
  atti.yaw   = angle + atti.yaw_cnt*360;
	return atti.yaw;
}


void EXTI4_IRQHandler(void)
{
	if(EXTI_GetITStatus(EXTI_Line4)==1)
	{
		if(imu_start_flag)
		{
			if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)
			{
				static BaseType_t xHigherPriorityTaskWoken;
				vTaskNotifyGiveFromISR(imu_Task_Handle,&xHigherPriorityTaskWoken);
				portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
			}
		}
	}
	EXTI_ClearITPendingBit(EXTI_Line4); 
}

float Get_imu_Pitch()
{
		return gimbal.sensor.pit_gyro_angle;
}










