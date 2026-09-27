#include "motor_task.h"
#include "bsp_can.h"
#include <string.h>

DamiaoMotor_t damiao_motor[DAMIAO_MOTOR_NUM];

/// @brief 电机初始化注册函数
/// @param
static void Motor_Init(void)
{
    DamiaoMotor_Init(&damiao_motor[0], DAMIAO_MOTOR_TYPE_J8006, DAMIAO_MODE_POS_SPEED, 1, 0x01);
    DamiaoMotor_SavePositionZero(&damiao_motor[0]);
    vTaskDelay(1);
    DamiaoMotor_Init(&damiao_motor[1], DAMIAO_MOTOR_TYPE_J4310, DAMIAO_MODE_POS_SPEED, 1, 0x02);
    DamiaoMotor_SavePositionZero(&damiao_motor[1]);
    vTaskDelay(1);
    DamiaoMotor_Init(&damiao_motor[2], DAMIAO_MOTOR_TYPE_S2325, DAMIAO_MODE_POS_SPEED, 1, 0x03);
    DamiaoMotor_SavePositionZero(&damiao_motor[2]);
    vTaskDelay(1);
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
        motor_control();
        vTaskDelay(5);
    }
}
