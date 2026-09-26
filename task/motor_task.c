#include "motor_task.h"
#include "bsp_can.h"

void motor_task(void *argument)
{
    Can1_Filter_Init(); // 初始化CAN1滤波器
    while (1)
    {
        
        vTaskDelay(5);
    }
}
