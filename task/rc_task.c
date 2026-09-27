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
    if (rc.RC_SBUS_t.ch4 >= 1750 && rc.RC_SBUS_t.ch4 <= 1850)
    {
        return 1; // 允许控制
    }
    else
    {
        return 0; // 禁止控制
    }
}

/// @brief 电机批量使能
/// @param
static void motor_enable(void)
{
    for (int i = 0; i < DAMIAO_MOTOR_NUM; i++)
    {
        DamiaoMotor_Enable(&damiao_motor[i]);
        vTaskDelay(1);
    }
}
/// @brief 电机批量失能
/// @param
static void motor_disable(void)
{
    for (int i = 0; i < DAMIAO_MOTOR_NUM; i++)
    {
        DamiaoMotor_Disable(&damiao_motor[i]);
        vTaskDelay(1);
    }
}

/// @brief 批量设置电机控制参数信息
/// @param
static void motor_set_control_msg(void)
{
    for (int i = 0; i < DAMIAO_MOTOR_NUM; i++)
    {
        damiao_motor[i].target_vel = 0.1f;
    }
    damiao_motor[0].target_pos = ((rc.RC_SBUS_t.ch10 - 992) * -0.01f) * 0.1f;
    damiao_motor[1].target_pos = ((rc.RC_SBUS_t.ch11 - 1014) * 0.01f) * 0.2f;
    damiao_motor[2].target_pos = ((rc.RC_SBUS_t.ch11 - 992) * 0.01f) * damiao_motor[2].reduction_ratio;
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
        /* 如果遥控器ch5拨杆拨下代表电机可用直接使能，未拨下代表不可用直接失能 */
        if (control_allow() == 1)
        {
            motor_enable();
        }
        else
        {
            motor_disable();
        }
        motor_set_control_msg();
        vofa_send(damiao_motor[1].target_pos, damiao_motor[1].out_angle, damiao_motor[1].out_rad);
        vTaskDelay(13); // 延时13ms
    }
    /* USER CODE END rc_task */
}
