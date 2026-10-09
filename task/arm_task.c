/**
 * @file arm_task.c
 * @author xioafo
 * @brief  机械臂末端位姿解算任务
 * @details 流程：等电机上线 -> 抓上电零位 -> 每 ARM_TELEMETRY_PERIOD_MS 解算
 *          一次并从 vofa_send 发出三个通道。
 *          任务优先级 osPriorityNormal1，低于 motor_task(osPriorityNormal2)，
 *          栈 1024 字节（Core/Src/freertos.c）——本函数内不要放大数组。
 * @note   验证方法（不需要卷尺）：
 *           1. ARM_TELEMETRY_MODE = 0，臂摆到标准直角位，应输出
 *                  ( (L2+L3)*1000 , (H+L1)*1000 , 0.0000 )   单位 mm / 度
 *           2. 锁定电机1/2、只慢扫电机0，sqrt(x^2 + (z-H)^2) 应恒定，等于
 *                  sqrt( (L2+L3)^2 + L1^2 ) * 1000
 *          这两条只证明代码没写错（cos/sin 颠倒、索引串位），证明不了长度参数
 *          正确；参数对不对必须拿卷尺量末端实际坐标比对。
 *          改了 arm_kinematics.h 第三节的几何宏以后，上面两个预期值要一起重算。
 */
#include "arm_task.h"
#include "arm_kinematics.h"
#include "vofa.h"

/**
 * @brief  机械臂正解任务主体：标定零位后周期性解算并发 VOFA
 * @param  argument FreeRTOS 任务入口形参，未使用
 */
void arm_task(void *argument)
{
    /* 等电机初始化、使能和第一批 CAN 反馈帧到位 */
    vTaskDelay(200);

    /* 零位必须在直角位抓取；电机没上线就一直等 */
    while (ArmKinematics_CalibrateZero() == 0)
    {
        vTaskDelay(50);
    }

    while (1)
    {
        ArmKinematics_Update();

#if (ARM_TELEMETRY_MODE == 1)
        /* 标定模式：把上电抓到的 O_k 发出来，抄进 arm_kinematics.h 的
         * ARM_ZERO_0/1/2，然后把 ARM_TELEMETRY_MODE 改回 0 */
        vofa_send(arm_zero_snapshot[0], arm_zero_snapshot[1], arm_zero_snapshot[2]);
#else
        /* 正常模式：x[mm] , z[mm] , theta[deg] */
        vofa_send(arm_pose.x * 1000.0f,
                  arm_pose.z * 1000.0f,
                  arm_pose.theta_deg);
#endif

        vTaskDelay(ARM_TELEMETRY_PERIOD_MS);
    }
}
