#include "damiao_motor.h"

#include <string.h>

static DamiaoMotor_t *g_motor_registry[DAMIAO_MOTOR_MAX_NUM];
static uint8_t g_motor_count = 0;

/**
 * @brief  初始化电机对象并注册进管理系统
 * @param  motor  电机结构体指针
 * @param  type   型号枚举 (DAMIAO_MOTOR_TYPE_S3519 / S2325 / J4310 / J8006 / J8009)
 * @param  bus_id CAN总线 (1或2)
 * @param  id     拨码地址 (1~8)
 */
void DamiaoMotor_Init(DamiaoMotor_t *motor, DamiaoMotorType_e type, uint8_t bus_id, uint8_t id)
{
    memset(motor, 0, sizeof(DamiaoMotor_t));
    motor->type = type;
    motor->bus_id = bus_id;
    motor->id = id;

    /* 1. 自动映射反馈 ID 和 减速比 */
    motor->rx_std_id = id; // 达妙电机反馈帧id为电机id
    if (type == DAMIAO_MOTOR_TYPE_S3519)
    {
        motor->reduction_ratio = REDUCTION_RATIO_S3519;
    }
    else if (type == DAMIAO_MOTOR_TYPE_S2325)
    {
        motor->reduction_ratio = REDUCTION_RATIO_S2325;
    }
    else if (type == DAMIAO_MOTOR_TYPE_J4310)
    {
        motor->reduction_ratio = REDUCTION_RATIO_J4310;
    }
    else if (type == DAMIAO_MOTOR_TYPE_J8006)
    {
        motor->reduction_ratio = REDUCTION_RATIO_J8006;
    }
    else if (type == DAMIAO_MOTOR_TYPE_J8009)
    {
        motor->reduction_ratio = REDUCTION_RATIO_J8009;
    }

    /* 2. 注册到管理器 */
    if (g_motor_count < DAMIAO_MOTOR_MAX_NUM)
    {
        g_motor_registry[g_motor_count++] = motor;
    }
}

/**
************************************************************************
* @brief:      	float_to_uint: 浮点数转换为无符号整数函数
* @param[in]:   x_float:	待转换的浮点数
* @param[in]:   x_min:		范围最小值
* @param[in]:   x_max:		范围最大值
* @param[in]:   bits: 		目标无符号整数的位数
* @retval:     	无符号整数结果
* @details:    	将给定的浮点数 x 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个指定位数的无符号整数
************************************************************************
**/
static int float_to_uint(float x_float, float x_min, float x_max, int bits)
{
    /* Converts a float to an unsigned int, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return (int)((x_float - offset) * ((float)((1 << bits) - 1)) / span);
}
/**
************************************************************************
* @brief:      	uint_to_float: 无符号整数转换为浮点数函数
* @param[in]:   x_int: 待转换的无符号整数
* @param[in]:   x_min: 范围最小值
* @param[in]:   x_max: 范围最大值
* @param[in]:   bits:  无符号整数的位数
* @retval:     	浮点数结果
* @details:    	将给定的无符号整数 x_int 在指定范围 [x_min, x_max] 内进行线性映射，映射结果为一个浮点数
************************************************************************
**/
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    /* converts unsigned int to float, given range and number of bits */
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}
/**
 * @brief  系统级接收处理函数 (在 CAN 接收中断中调用)
 * @param  bus_id  产生中断的 CAN 总线 (1或2)
 * @param  std_id  接收到的标准帧 ID
 * @param  rx_data 接收到的8字节数据缓冲区指针
 */
void DamiaoMotorManager_RxHandler(uint8_t bus_id, uint8_t std_id, const uint8_t *rx_data)
{
    for (uint8_t i = 0; i < g_motor_count; i++)
    {
        DamiaoMotor_t *m = g_motor_registry[i];

        if (m->bus_id == bus_id && m->rx_std_id == ((rx_data[0]) & 0x0F))
        {
            m->is_online = 1;

            /* 解析反馈数据 */
            m->fdb_id = (rx_data[0]) & 0x0F;
            m->state = (rx_data[0]) >> 4;
            m->p_int = (rx_data[1] << 8) | rx_data[2];
            m->v_int = (rx_data[3] << 4) | (rx_data[4] >> 4);
            m->t_int = ((rx_data[4] & 0xF) << 8) | rx_data[5];
            m->pos = uint_to_float(m->p_int, P_MIN, P_MAX, 16); // (-12.5,12.5)
            m->vel = uint_to_float(m->v_int, V_MIN, V_MAX, 12); // (-45.0,45.0)
            m->tor = uint_to_float(m->t_int, T_MIN, T_MAX, 12); // (-10.0,10.0)
            m->Tmos = (float)(rx_data[6]);
            m->Tcoil = (float)(rx_data[7]);

            /* 处理多圈累加逻辑 (过零点判断) */
            if (!m->is_first_recv)
            {
                m->last_pos = m->pos;
                m->is_first_recv = 1;
            }
            else
            {
                float diff = m->pos - m->last_pos;
                if (diff >= 12.5f)
                    diff -= 25.0f;
                else if (diff <= -12.5f)
                    diff += 25.0f;
                m->total_rad += diff;
                m->last_pos = m->pos;
            }

            /* 核心计算：物理量刷新到结构体成品变量 */
            if (m->reduction_ratio > 0.0f)
            {
                // 计算输出轴 RPM
                m->out_rpm = m->vel / m->reduction_ratio;
                // 计算输出轴弧度
                m->out_rad = m->total_rad / m->reduction_ratio;
                // 计算输出轴角度 (累加值)
                m->out_angle = (m->out_rad / DAMIAO_PI) * 180.0f;
            }
            break;
        }
    }
}

void DamiaoMotor_Enable(DamiaoMotor_t *motor)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
    uint32_t id = motor->id;
    User_CAN_Transmit(motor->bus_id, id, data, 8);
}

void DamiaoMotor_Disable(DamiaoMotor_t *motor)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFD};
    uint32_t id = motor->id;
    User_CAN_Transmit(motor->bus_id, id, data, 8);
}

void DamiaoMotor_SavePositionZero(DamiaoMotor_t *motor)
{
    uint8_t data[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFE};
    uint32_t id = motor->id;
    User_CAN_Transmit(motor->bus_id, id, data, 8);
}

void DamiaoMotor_MotionControl(DamiaoMotor_t *motor)
{
    uint8_t data[8];
    uint16_t pos_tmp, vel_tmp, kp_tmp, kd_tmp, tor_tmp;
    uint32_t id = DAMIAO_MIT_MODE_TXHEADER + motor->id;

    pos_tmp = float_to_uint(motor->target_pos, P_MIN, P_MAX, 16);
    vel_tmp = float_to_uint(motor->target_vel, V_MIN, V_MAX, 12);
    kp_tmp = float_to_uint(motor->target_kp, KP_MIN, KP_MAX, 12);
    kd_tmp = float_to_uint(motor->target_kd, KD_MIN, KD_MAX, 12);
    tor_tmp = float_to_uint(motor->target_tor, T_MIN, T_MAX, 12);

    data[0] = (pos_tmp >> 8);
    data[1] = pos_tmp;
    data[2] = (vel_tmp >> 4);
    data[3] = ((vel_tmp & 0xF) << 4) | (kp_tmp >> 8);
    data[4] = kp_tmp;
    data[5] = (kd_tmp >> 4);
    data[6] = ((kd_tmp & 0xF) << 4) | (tor_tmp >> 8);
    data[7] = tor_tmp;
    User_CAN_Transmit(motor->bus_id, id, data, 8);
}

void DamiaoMotor_PositionSpeedControl(DamiaoMotor_t *motor)
{
    uint32_t id = DAMIAO_POS_SPEED_MODE_TXHEADER + motor->id;
    uint8_t data[8];

    uint8_t *pbuf, *vbuf;
    pbuf = (uint8_t *)&motor->target_pos;
    vbuf = (uint8_t *)&motor->target_vel;

    data[0] = *pbuf;
    data[1] = *(pbuf + 1);
    data[2] = *(pbuf + 2);
    data[3] = *(pbuf + 3);
    data[4] = *vbuf;
    data[5] = *(vbuf + 1);
    data[6] = *(vbuf + 2);
    data[7] = *(vbuf + 3);
    User_CAN_Transmit(motor->bus_id, id, data, 8);
}

void DamiaoMotor_SpeedControl(DamiaoMotor_t *motor)
{
    uint32_t id = DAMIAO_SPEED_MODE_TXHEADER + motor->id;
    uint8_t data[4];

    uint8_t *vbuf;
    vbuf = (uint8_t *)&motor->target_vel;

    data[0] = *vbuf;
    data[1] = *(vbuf + 1);
    data[2] = *(vbuf + 2);
    data[3] = *(vbuf + 3);
    User_CAN_Transmit(motor->bus_id, id, data, 4);
}
