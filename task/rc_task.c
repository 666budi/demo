#include "rc_task.h"
#include "motor_task.h"
#include "damiao_motor.h"
#include "vofa.h"
#include <string.h>

RC_t rc; // 定义遥控器数据结构体

/// @brief 检查控制允许状态
/// @param None
static uint8_t control_allow(void)
{
    // 检查遥控器开关状态，允许控制
    if (rc.RC_DBUS_t.sw1 == 3)
    {
        return 1; // 允许控制
    }
    else
    {
        return 0; // 禁止控制
    }
}

/// @brief 电机批量失能
/// @param
static void motor_disable(void)
{
    for (int i = 0; i < DAMIAO_MOTOR_NUM; i++)
    {
        DamiaoMotor_Disable(&damiao_motor[i]);
        vTaskDelay(2);
    }
}

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
    damiao_motor[0].target_pos = ((rc.RC_DBUS_t.ch3 - 1024) / 100.0) * -0.1f;
    damiao_motor[0].target_pos = max_pos(damiao_motor[0].target_pos, 0.8f, -0.8f);

    damiao_motor[1].target_vel = 0.5f;
    damiao_motor[1].target_pos = ((rc.RC_DBUS_t.ch1 - 1024) / 100.0) * 0.1f;
    damiao_motor[1].target_pos = max_pos(damiao_motor[1].target_pos, 2.0f, -0.8f);

    damiao_motor[2].target_vel = 3.0f;
    damiao_motor[2].target_pos += ((rc.RC_DBUS_t.ch0 - 1024) * 0.01f) * damiao_motor[2].reduction_ratio * 0.01f;
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
        if (control_allow())
        {
            motor_set_control_msg();
        }
        else
        {
            if(rc.RC_DBUS_t.sw2 == 3)
            {
                motor_disable();
            }
        }
        vTaskDelay(20); // 延时20ms
    }
    /* USER CODE END rc_task */
}
