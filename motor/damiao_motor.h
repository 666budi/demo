#ifndef __DAMIAO_MOTOR_H
#define __DAMIAO_MOTOR_H

#include "stm32f4xx_hal.h"
#include "can.h"

/* ==================== 达妙电机模式帧头定义 ==================== */
#define DAMIAO_MIT_MODE_TXHEADER 0x000
#define DAMIAO_POS_SPEED_MODE_TXHEADER 0x100
#define DAMIAO_SPEED_MODE_TXHEADER 0x200

/* ==================== 物理常量定义 ==================== */
#define DAMIAO_PI 3.14159265358979f
#define DAMIAO_ENC_RESOLUTION 16384.0f // 编码器分辨率(14位)，S系列为增量式编码器，J系列为双绝对值编码器

#define P_MIN -12.5f  // 位置最小值
#define P_MAX 12.5f   // 位置最大值
#define V_MIN -45.0f  // 速度最小值
#define V_MAX 45.0f   // 速度最大值
#define KP_MIN 0.0f   // KP最小值
#define KP_MAX 500.0f // KP最大值
#define KD_MIN 0.0f   // KD最小值
#define KD_MAX 5.0f   // KD最大值
#define T_MIN -10.0f  // 扭矩最小值
#define T_MAX 10.0f   // 扭矩最大值

/* ==================== 电机参数定义 ==================== */
/* 达妙S3519电机参数定义 */
#define REDUCTION_RATIO_S3519 19.203208556149f // 减速比3591/187
#define MAX_RPM_S3519 395.0f                   // 输出轴额定转速

/* 达妙S2325电机参数定义 */
#define REDUCTION_RATIO_S2325 25.0f // 减速比
#define MAX_RPM_S2325 380.0f        // 输出轴额定转速

/* 达妙J4310电机参数定义 */
#define REDUCTION_RATIO_J4310 10.0f // 减速比
#define MAX_RPM_J4310 120.0f        // 输出轴额定转速

/* 达妙J8006电机参数定义 */
#define REDUCTION_RATIO_J8006 6.0f // 减速比
#define MAX_RPM_J8006 120.0f       // 输出轴额定转速

/* 达妙J8009电机参数定义 */
#define REDUCTION_RATIO_J8009 9.0f // 减速比
#define MAX_RPM_J8009 100.0f       // 输出轴额定转速

#define DAMIAO_MOTOR_MAX_NUM 16 // 最大支持的电机数量

/* ==================== 电机类型枚举定义 ==================== */
/* 电机类型枚举 */
typedef enum
{
    DAMIAO_MOTOR_TYPE_S3519 = 0,
    DAMIAO_MOTOR_TYPE_S2325 = 1,
    DAMIAO_MOTOR_TYPE_J4310 = 2,
    DAMIAO_MOTOR_TYPE_J8006 = 3,
    DAMIAO_MOTOR_TYPE_J8009 = 4
} DamiaoMotorType_e;

/* 电机模式枚举 */
typedef enum
{
    DAMIAO_MODE_MIT = 0,
    DAMIAO_MODE_POS_SPEED = 1,
    DAMIAO_MODE_SPEED = 2
} DamiaoMotorMode_e;

/* ==================== 电机数据结构定义 ==================== */
/* 达妙电机数据结构 */
typedef struct DamiaoMotor_s
{
    /* --- 静态配置 --- */
    DamiaoMotorType_e type; // 电机型号
    DamiaoMotorMode_e mode; // 电机工作模式
    uint8_t bus_id;         // CAN 总线 ID (1 或 2)
    uint8_t id;             // 电机 ID
    uint8_t rx_std_id;      // 自动计算的反馈帧 ID
    float reduction_ratio;  // 自动加载的减速比

    /* --- 状态数据 (底层原始) --- */
    uint8_t is_online;     // 在线状态标志位
    uint8_t is_first_recv; // 初始化标志位
    uint8_t fdb_id;        // 反馈 ID
    uint8_t state;         // 电机状态
    uint16_t p_int;        // 位置值(red/s)
    uint16_t v_int;        // 速度值(red/s)
    uint16_t t_int;        // 扭矩值
    uint16_t kp_int;       // Kp值
    uint16_t kd_int;       // Kd值

    /* --- 成品输出数据 (用户直接读取此处) --- */
    float pos;       // 实际位置值
    float vel;       // 实际速度值
    float tor;       // 实际扭矩值
    float Kp;        // 实际Kp值
    float Kd;        // 实际Kd值
    float Tmos;      // 驱动上 MOS 的平均温度，单位℃
    float Tcoil;     // 驱动上线圈的平均温度，单位℃
    float last_pos;  // 上一次位置值
    float total_rad; // 多圈连续弧度累加
    float out_angle; // 输出轴实际角度 (度, -inf ~ +inf)
    float out_rad;   // 输出轴实际弧度 (Rad, -inf ~ +inf)
    float out_rpm;   // 输出轴实际转速 (RPM)

    /* --- 控制接口 --- */
    float target_vel;  // 用户直接修改此值来控制速度
    float target_pos;  // 用户直接修改此值来控制位置
    float target_tor;  // 用户直接修改此值来控制扭矩
    float target_kp;   // 用户直接修改此值来控制Kp
    float target_kd;   // 用户直接修改此值来控制Kd
    uint8_t is_enable; // 使能开关

} DamiaoMotor_t;

/* ==================== 接口函数 ==================== */

/**
 * @brief  初始化电机对象并注册进管理系统
 * @param  motor  电机结构体指针
 * @param  type   型号枚举 (DAMIAO_MOTOR_TYPE_S3519 / S2325 / J4310 / J8006 / J8009)
 * @param  mode   工作模式 (DAMIAO_MODE_MIT / DAMIAO_MODE_POS_SPEED / DAMIAO_MODE_SPEED)
 * @param  bus_id CAN总线 (1或2)
 * @param  id     电机 ID
 */
void DamiaoMotor_Init(DamiaoMotor_t *motor, DamiaoMotorType_e type, DamiaoMotorMode_e mode, uint8_t bus_id, uint8_t id);

/**
 * @brief  系统级接收处理函数 (在 CAN 接收中断中调用)
 * @param  bus_id  产生中断的 CAN 总线 (1或2)
 * @param  std_id  接收到的标准帧 ID
 * @param  rx_data 接收到的8字节数据缓冲区指针
 */
void DamiaoMotorManager_RxHandler(uint8_t bus_id, uint8_t std_id, const uint8_t *rx_data);

/**
 * @brief  电机使能函数
 * @param  motor  电机结构体指针
 */
void DamiaoMotor_Enable(DamiaoMotor_t *motor);

/**
 * @brief  电机失能函数
 * @param  motor  电机结构体指针
 */
void DamiaoMotor_Disable(DamiaoMotor_t *motor);

/**
 * @brief  电机位置设零函数
 * @param  motor  电机结构体指针
 */
void DamiaoMotor_SavePositionZero(DamiaoMotor_t *motor);

/**
 * @brief  电机MIT模式控制函数
 * @param  motor 电机结构体指针
 */
void DamiaoMotor_MotionControl(DamiaoMotor_t *motor);

/**
 * @brief  电机位置速度模式控制函数
 * @param  motor 电机结构体指针
 */
void DamiaoMotor_PositionSpeedControl(DamiaoMotor_t *motor);

/**
 * @brief  电机速度模式控制函数
 * @param  motor 电机结构体指针
 */
void DamiaoMotor_SpeedControl(DamiaoMotor_t *motor);

/**
 * @brief  底层硬件发送接口回调 (需用户自行实现)
 * @param  bus_id  CAN总线 (1或2)
 * @param  std_id  需发送的标准帧 ID
 * @param  data    数据指针
 * @param  len     数据长度 (固定为8)
 */
extern void User_CAN_Transmit(uint8_t bus_id, uint32_t std_id, uint8_t *data, uint8_t len);

#endif
