#include "motor_task.h"
#include "rc_task.h"
#include "bsp_can.h"
#include <string.h>

DamiaoMotor_t damiao_motor[DAMIAO_MOTOR_NUM];

/// @brief 电机初始化注册函数
/// @param
static void Motor_Init(void)
{
    taskENTER_CRITICAL();
    DamiaoMotor_Init(&damiao_motor[0], DAMIAO_MOTOR_TYPE_J8006, DAMIAO_MODE_POS_SPEED, 1, 0x01);
    DamiaoMotor_SavePositionZero(&damiao_motor[0]);
    DamiaoMotor_Init(&damiao_motor[1], DAMIAO_MOTOR_TYPE_J4310, DAMIAO_MODE_POS_SPEED, 1, 0x02);
    DamiaoMotor_SavePositionZero(&damiao_motor[1]);
    DamiaoMotor_Init(&damiao_motor[2], DAMIAO_MOTOR_TYPE_S2325, DAMIAO_MODE_POS_SPEED, 1, 0x03);
    DamiaoMotor_SavePositionZero(&damiao_motor[2]);
    taskEXIT_CRITICAL();
}

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
    taskENTER_CRITICAL();
    for (int i = 0; i < DAMIAO_MOTOR_NUM; i++)
    {
        DamiaoMotor_Enable(&damiao_motor[i]);
    }
    taskEXIT_CRITICAL();
}
/// @brief 电机批量失能
/// @param
static void motor_disable(void)
{
    taskENTER_CRITICAL();
    for (int i = 0; i < DAMIAO_MOTOR_NUM; i++)
    {
        DamiaoMotor_Disable(&damiao_motor[i]);
    }
    taskEXIT_CRITICAL();
}

/// @brief 批量发送电机控制信号
/// @param
static void motor_control(void)
{
    for (int i = 0; i < DAMIAO_MOTOR_NUM; i++)
    {
        /* 根据电机的模式选择不同的数据协议发送 */
        switch (damiao_motor[i].mode)
        {
        case DAMIAO_MODE_MIT:
            DamiaoMotor_MotionControl(&damiao_motor[i]);
            break;
        case DAMIAO_MODE_POS_SPEED:
            DamiaoMotor_PositionSpeedControl(&damiao_motor[i]);
            break;
        case DAMIAO_MODE_SPEED:
            DamiaoMotor_SpeedControl(&damiao_motor[i]);
            break;
        default:
            break;
        }
    }
}

void motor_task(void *argument)
{
    Can1_Filter_Init(); // 初始化CAN1滤波器
    vTaskDelay(5);
    Motor_Init();
    vTaskDelay(5);
    while (1)
    {
        /* 如果遥控器ch5拨杆拨下代表电机可用直接使能，未拨下代表不可用直接失能 */
        if (control_allow() == 1 && damiao_motor[0].is_enable == 0)
        {
            motor_enable();
        }
        else if (control_allow() == 0 && damiao_motor[0].is_enable == 1)
        {
            motor_disable();
        }
        motor_control();
        vTaskDelay(5);
    }
}
