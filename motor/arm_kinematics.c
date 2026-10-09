/**
 * @file arm_kinematics.c
 * @author xioafo
 * @brief  三自由度机械臂末端位姿正解
 * @details 实现 arm_kinematics.h 里描述的两级闭式解算：
 *            第 1 级  out_rad -> 关节角 q（ArmKinematics_Update 内完成）
 *            第 2 级  q -> 末端位姿 (x, z, theta)（ArmKinematics_Solve 内完成）
 *          本文件只做纯数学换算，不发送 CAN、不操作电机。
 */
#include "arm_kinematics.h"
#include "motor_task.h"
#include <math.h>

/* ==========================================================================
 *  运行期参数表：初值来自 arm_kinematics.h 里的宏
 *  刻意不加 const，方便在调试器 Watch 窗口里实时改
 * ========================================================================== */
ArmParams_t arm_params =
{
    ARM_H,
    ARM_L1, ARM_L2, ARM_L3,
    { ARM_RATIO_0, ARM_RATIO_1, ARM_RATIO_2 },
    { ARM_DIR_0,   ARM_DIR_1,   ARM_DIR_2   },
    ARM_DELTA,
    ARM_PHI1_0, ARM_PHI2_0, ARM_PHI3_0,
    ARM_TCP_LX, ARM_TCP_LZ
};

/* ==========================================================================
 *  上电零位 O_k
 *  为什么必须是运行期变量而不是宏：out_rad 的零点不是机械绝对基准。
 *  total_rad 从"第一帧反馈"起算（damiao_motor.c 里第一帧只赋 last_pos
 *  不累加），而 DamiaoMotor_SavePositionZero() 几乎同时在开机时下发，
 *  两者先后决定 total_rad 在标定时刻是 0 还是 -X。对 S 系列（电机2 是
 *  S2325，增量式编码器）这个零点更是只对应"上电那一瞬间"。
 *  所以 O_k 每次上电实测；宏里存的是"预期值"，只用来做自检。
 * ========================================================================== */
float   arm_zero_snapshot[3]  = { 0.0f, 0.0f, 0.0f };
float   arm_zero_deviation[3] = { 0.0f, 0.0f, 0.0f };
uint8_t arm_zero_ok = 0;

/* 最新一次解算结果，供 arm_task 读取 */
ArmPose_t arm_pose;

/* 宏里的零位预期值，只用于上电自检比对 */
static const float arm_zero_expect[3] = { ARM_ZERO_0, ARM_ZERO_1, ARM_ZERO_2 };

/* 是否已完成上电零位标定 */
static uint8_t arm_calibrated = 0;

/* 传动比容错：调试器里误改成 0 会产生 inf/nan，静默污染全部输出 */
#define ARM_RATIO_SAFE(r) ((((r) > -1.0e-6f) && ((r) < 1.0e-6f)) ? 1.0f : (r))

/**
 * @brief 把 arm_params 全部恢复成 arm_kinematics.h 里那些宏的出厂默认值
 * @note  调试器里把 ratio/dir 调乱后用这个一键还原，不用复位
 */
void ArmKinematics_ResetParams(void)
{
    arm_params.h  = ARM_H;
    arm_params.l1 = ARM_L1;
    arm_params.l2 = ARM_L2;
    arm_params.l3 = ARM_L3;

    arm_params.ratio[0] = ARM_RATIO_0;
    arm_params.ratio[1] = ARM_RATIO_1;
    arm_params.ratio[2] = ARM_RATIO_2;

    arm_params.dir[0] = ARM_DIR_0;
    arm_params.dir[1] = ARM_DIR_1;
    arm_params.dir[2] = ARM_DIR_2;

    arm_params.delta = ARM_DELTA;
    arm_params.phi10 = ARM_PHI1_0;
    arm_params.phi20 = ARM_PHI2_0;
    arm_params.phi30 = ARM_PHI3_0;

    arm_params.tcp_lx = ARM_TCP_LX;
    arm_params.tcp_lz = ARM_TCP_LZ;
}

/**
 * @brief  上电零位标定，抓取三个电机的 out_rad 快照作为 O_k
 * @return 1 = 三个电机都 is_online 且已收到第一帧，快照已抓取
 *         0 = 条件未满足，调用方应延时重试
 * @note   必须在臂处于标准直角位时调用；运行期不要再次调用，会把零位搬走
 */
uint8_t ArmKinematics_CalibrateZero(void)
{
    float   m[3];
    uint8_t i;

    /* 三个电机必须都在线、且都已收到第一帧，否则此时 out_rad 还没有意义 */
    for (i = 0; i < 3; i++)
    {
        if (damiao_motor[i].is_online == 0)
        {
            return 0;
        }
        if (damiao_motor[i].is_first_recv == 0)
        {
            return 0;
        }
    }

    /* out_rad 在 CAN 接收中断里更新，取快照必须进临界区 */
    taskENTER_CRITICAL();
    for (i = 0; i < 3; i++)
    {
        m[i] = damiao_motor[i].out_rad;
    }
    taskEXIT_CRITICAL();

#if (ARM_USE_FIXED_ZERO == 1)
    /* 用宏里的固定零位，不采信本次读数 */
    for (i = 0; i < 3; i++)
    {
        arm_zero_snapshot[i]  = arm_zero_expect[i];
        arm_zero_deviation[i] = 0.0f;
    }
    arm_zero_ok = 1;
#else
    /* 采信本次读数，并与预期值比对 */
    arm_zero_ok = 1;
    for (i = 0; i < 3; i++)
    {
        arm_zero_snapshot[i]  = m[i];
        arm_zero_deviation[i] = m[i] - arm_zero_expect[i];

        if ((arm_zero_deviation[i] > ARM_ZERO_TOL) ||
            (arm_zero_deviation[i] < -ARM_ZERO_TOL))
        {
            /* 不阻止运行，只把标志置 0 让人能看见：
             * 上电时臂不在直角位，整条解算链的零位都是错的 */
            arm_zero_ok = 0;
        }
    }
#endif

    arm_calibrated = 1;
    return 1;
}

/**
 * @brief 读一次三路 out_rad，算出关节角并更新 arm_pose，建议放在周期任务里调用
 * @note  out_rad 在 CAN 接收中断里被改写，读取必须整体包在临界区内，
 *        否则可能拿到三个轴来自不同时刻的错配数据
 */
void ArmKinematics_Update(void)
{
    float   m[3];
    float   q[3];
    uint8_t i;
    uint8_t online = 1;

    taskENTER_CRITICAL();
    for (i = 0; i < 3; i++)
    {
        m[i] = damiao_motor[i].out_rad;
        if (damiao_motor[i].is_online == 0)
        {
            online = 0;
        }
    }
    taskEXIT_CRITICAL();

    /* 第 1 级：电机输出轴角 -> 关节角 */
    for (i = 0; i < 3; i++)
    {
        q[i] = arm_params.dir[i] * (m[i] - arm_zero_snapshot[i])
               / ARM_RATIO_SAFE(arm_params.ratio[i]);
    }

    /* 第 2 级：关节角 -> 末端位姿 */
    ArmKinematics_Solve(q[0], q[1], q[2], &arm_pose);

    /* 只有标定完成且三个电机都在线，结果才可信 */
    arm_pose.valid   = (uint8_t)((arm_calibrated != 0) && (online != 0));
    arm_pose.zero_ok = arm_zero_ok;
}

/**
 * @brief  纯函数：由三个关节角算出末端位姿，不读电机、不写 valid
 * @param  q1  肩关节角 (rad)，0 = 直角零位
 * @param  q2  肘关节角 (rad)，0 = 直角零位
 * @param  q3  腕关节角 (rad)，0 = 直角零位
 * @param  out 解算结果输出，不可为 NULL
 * @note   相对角串链：每一级都从上一根连杆的绝对角继续累加，
 *         所以转 q1 时三根连杆一起转、转 q3 时只有末端转
 */
void ArmKinematics_Solve(float q1, float q2, float q3, ArmPose_t *out)
{
    float phi1, phi2, phi3;
    float x3, z3;
    float ct, st;

    /* 相对角串链：每一级都从上一根连杆的绝对角继续累加 */
    phi1 = arm_params.phi10 + q1;
    phi2 = arm_params.phi20 + q1 + q2;
    phi3 = arm_params.phi30 + q1 + q2 + q3;

    out->q[0]   = q1;
    out->q[1]   = q2;
    out->q[2]   = q3;
    out->phi[0] = phi1;
    out->phi[1] = phi2;
    out->phi[2] = phi3;

    /* 三连杆求和：末端尖端 = 腕轴点 + 末端连杆，y 方向恒为 0 */
    x3 = arm_params.l1 * cosf(phi1)
       + arm_params.l2 * cosf(phi2)
       + arm_params.l3 * cosf(phi3);

    z3 = arm_params.h
       + arm_params.l1 * sinf(phi1)
       + arm_params.l2 * sinf(phi2)
       + arm_params.l3 * sinf(phi3);

    out->theta     = phi3 + arm_params.delta;
    out->theta_deg = out->theta * (180.0f / DAMIAO_PI);

    /* 工具坐标偏置：在末端坐标系里再偏一段（LX 沿末端轴向，LZ 垂直末端轴向） */
    ct = cosf(out->theta);
    st = sinf(out->theta);

    out->x = x3 + arm_params.tcp_lx * ct - arm_params.tcp_lz * st;
    out->z = z3 + arm_params.tcp_lx * st + arm_params.tcp_lz * ct;
    out->y = 0.0f;
}
