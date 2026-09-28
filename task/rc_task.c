#include "rc_task.h"
#include "motor_task.h"
#include "damiao_motor.h"
#include "vofa.h"
#include <string.h>

RC_t rc; // 定义遥控器数据结构体

/// @brief 控制极限位置位置限定
/// @param target 目标位置
/// @param max    最大位置
/// @param min    最小位置
/// @return
static float max_pos(float target, float max, float min)
{
    if (target >= max)
    {
        return max;
    }
    else if (target <= min)
    {
        return min;
    }
    else
    {
        return target;
    }
}

/// @brief 批量设置电机控制参数信息
/// @param
static void motor_set_control_msg(void)
{
    taskENTER_CRITICAL();
    damiao_motor[0].target_vel = 0.3f;
    damiao_motor[0].target_pos = ((rc.RC_SBUS_t.ch10 - 992) / 100.0) * -0.1f;
    damiao_motor[0].target_pos = max_pos(damiao_motor[0].target_pos, 0.8f, -0.8f);

    damiao_motor[1].target_vel = 0.5f;
    damiao_motor[1].target_pos = ((rc.RC_SBUS_t.ch11 - 992) / 100.0) * 0.1f;
    damiao_motor[1].target_pos = max_pos(damiao_motor[1].target_pos, 2.0f, -0.8f);

    damiao_motor[2].target_vel = 3.0f;
    damiao_motor[2].target_pos += ((rc.RC_SBUS_t.ch1 - 998) * 0.01f) * damiao_motor[2].reduction_ratio * 0.01f;
    damiao_motor[2].target_pos = max_pos(damiao_motor[2].target_pos, 6.3f * damiao_motor[2].reduction_ratio, -6.3f * damiao_motor[2].reduction_ratio);
    taskEXIT_CRITICAL();
}

void rc_task(void *argument)
{
    /* USER CODE BEGIN rc_task */
    vofa_init();
    RC_Init();                    // 初始化遥控器接收
    memset(&rc, 0, sizeof(RC_t)); // 清零遥控器数据结构体
    vTaskDelay(50);               // 延时50ms等待遥控器数据稳定
    /* Infinite loop */
    while (1)
    {
        rc = RC_GetInfo(); // 获取遥控器数据结构体的快照
        vTaskDelay(1);
        motor_set_control_msg();
        vofa_send(damiao_motor[0].target_pos, damiao_motor[0].out_angle, damiao_motor[0].out_rad);
        vTaskDelay(30); // 延时30ms
    }
    /* USER CODE END rc_task */
}
