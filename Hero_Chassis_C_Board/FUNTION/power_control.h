#ifndef __POWER_CONTROL_H__
#define __POWER_CONTROL_H__

#include "stm32f4xx.h"
#include "chassis_task.h"

#define Debug_Power				70//没有裁判系统时的功率限制
#define Deta_Power				60//触发缓启动的功率差
#define CAP_LOW						11//低于14V不能加速
#define BUFFER_ENERGY_REF               55//缓冲焦耳期望
extern float Charge_factor;


float Chassis_Power_Control(chassis_wheel_t *chassis_power_control);


#endif 
