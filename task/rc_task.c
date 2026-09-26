#include "rc_task.h"
#include "motor_task.h"
#include "string.h"

RC_t rc; // 定义遥控器数据结构体

/// @brief 检查控制允许状态
/// @param None
static uint8_t control_allow(void)
{
    // 检查遥控器开关状态，允许控制
    if (rc.RC_SBUS_t.ch4 >= 1700 && rc.RC_SBUS_t.ch4 <= 1800)
    {
        return 1; // 允许控制
    }
    else
    {
        return 0; // 禁止控制
    }
}
void rc_task(void *argument)
{
    /* USER CODE BEGIN rc_task */
    RC_Init();                    // 初始化遥控器接收
    memset(&rc, 0, sizeof(RC_t)); // 清零遥控器数据结构体

    vTaskDelay(50); // 延时50ms等待遥控器数据稳定
    /* Infinite loop */
    while (1)
    {
        rc = RC_GetInfo();        // 获取遥控器数据结构体的快照
        vTaskDelay(1);            // 延时1ms
        if (control_allow() == 1) // 检查是否允许控制
        {
        }
        else
        {
        }
        vTaskDelay(10); // 延时10ms
    }
    /* USER CODE END rc_task */
}
