/**
 * @file vofa.c
 * @brief VOFA+ 上位机数据发送 —— FireWater 波形协议（纯 ASCII，无 printf）
 * @note  FireWater 帧格式: ch0,ch1,ch2\r\n
 *        例: -1.2345,0.0000,12.5000\r\n
 *
 * @details 本文件原先使用 printf("%f,%f,%f\n") 打印。因为 float 在可变参数中会
 *          被提升为 double，而 Cortex-M4F 只有单精度 FPU，链接器为此选中了
 *          MicroLIB 的完整版浮点 printf(printfa.o)，把 _printf_core(136B) +
 *          _fp_digits(64B) + 软件双精度模拟(dadd/dmul 各 48B)全部拉进调用链，
 *          实测压栈约 370 字节；而 rc_task 栈只有 512 字节，扣除任务初始帧与
 *          PendSV 上下文保存后可用量不足 400 字节，再叠加任意一次中断入栈即
 *          溢出，溢出踩到 heap_4 中相邻的任务栈/TCB，表现为"运行一段时间后
 *          随机卡死"。此处改为无库依赖的精确定点转换 + 单次成帧发送，彻底断开
 *          printf 依赖，同时保持 ASCII 文本的可读性。
 */
#include "vofa.h"
#include <stdint.h>
#include <string.h> /* memcpy：仅用于取 float 的位模式，避免严格别名问题 */
#include <stdio.h>  /* 仅用于保留 fputc 重定向，不再使用 printf */
#include "usart.h"

uint8_t vofa_rx_buffer[16];

/*
 * 编译期约束：小数位权重表是按 4 位手写的，位数或倍数被改动时必须同步，
 * 否则会静默少打/多打小数位，故直接在编译期挡掉。
 */
#if (VOFA_FRAC_DIGITS != 4)
#error "VOFA_FRAC_DIGITS 必须与 vofa.c 中 frac_unit 权重表的项数一致（当前为 4）"
#endif
#if (VOFA_FRAC_SCALE != 10000u)
#error "VOFA_FRAC_SCALE 必须等于 10^VOFA_FRAC_DIGITS（4 位小数时应为 10000）"
#endif

/** @brief 一行 FireWater 报文的字符缓冲（静态分配，不占任务栈） */
static char vofa_line_buf[VOFA_LINE_MAX_LEN];

/* ===========================================================================
 * 浮点 -> FireWater ASCII 文本
 * 只使用位运算与 64 位整数，无浮点乘法舍入、无递归、无库函数、栈占用固定
 * ========================================================================= */

/**
 * @brief  精确计算 round(|x| * 10^VOFA_FRAC_DIGITS)，直接由 IEEE-754 位模式推算
 * @param  abs_v  已取绝对值并限幅后的输入（>= 0，有限）
 * @retval 放大 VOFA_FRAC_SCALE 倍并四舍五入后的整数
 *
 * @details abs_v 可无损写成  m * 2^e  的形式：m 是 24 位有效数字(含隐藏位)，
 *          e 是指数。于是
 *              round(abs_v * 1e4) = round(m * 1e4 * 2^e)
 *          e>=0 时左移即精确结果；e<0 时右移并用"加半个最低位"完成四舍五入。
 *          全程只有整数移位和加法，不存在任何浮点舍入，因此与 printf("%.4f")
 *          的十进制结果逐位一致。
 *
 *          对比：先算 frac*2^24 再转整数的做法是错的——当 abs_v 落在 [0.25,0.5)
 *          时其 ulp 为 2^-25，乘 2^24 后可能出现 x.5，被 (uint32_t) 截断而系统性
 *          偏小（实测 0.44955f 会错成 0.4495）。故改为按位模式精确换算。
 */
static uint32_t vofa_scale_round(float abs_v)
{
    uint32_t bits;
    uint64_t m;
    uint64_t num;
    int32_t e;
    uint32_t k;

    (void)memcpy(&bits, &abs_v, sizeof(bits));
    /* 指数域为 0：值为 0 或次正规数(< 2^-126)，乘 1e4 后远小于 0.5，舍入为 0 */
    if ((bits & 0x7F800000u) == 0u)
    {
        return 0u;
    }

    m = (uint64_t)(bits & 0x007FFFFFu) | 0x00800000u;      /* m in [2^23, 2^24) */
    e = (int32_t)((bits >> 23) & 0xFFu) - 127 - 23;        /* abs_v = m * 2^e */

    num = m * (uint64_t)VOFA_FRAC_SCALE;                   /* < 2^24 * 1e4 < 2^38 */

    if (e >= 0)
    {
        /* 输入已限幅到 VOFA_ABS_MAX，此分支实际不可达，仅作防御 */
        if (e > 25)
        {
            return 2000000000u; /* = VOFA_ABS_MAX * VOFA_FRAC_SCALE，饱和上限 */
        }
        return (uint32_t)(num << (uint32_t)e);             /* 精确，无需舍入 */
    }

    k = (uint32_t)(-e);
    if (k >= 40u) /* num < 2^38，右移这么多必然舍入为 0 */
    {
        return 0u;
    }
    return (uint32_t)((num + (1ULL << (k - 1u))) >> k);    /* 四舍五入 */
}

/**
 * @brief  把一个浮点数写入缓冲区，形如 -12.3456
 * @param  p      当前写入位置
 * @param  end    缓冲区安全末尾（不含），用于越界保护
 * @param  value  待转换值；超量程/±Inf 按饱和输出，NaN 按 0 输出
 * @retval 新的写入位置
 */
static char *vofa_put_float(char *p, const char *end, float value)
{
    /* 小数位权重表，长度与 VOFA_FRAC_DIGITS 一致：10^(N-1) .. 10^0 */
    static const uint32_t frac_unit[VOFA_FRAC_DIGITS] = {
        (VOFA_FRAC_SCALE / 10u),    (VOFA_FRAC_SCALE / 100u),
        (VOFA_FRAC_SCALE / 1000u),  (VOFA_FRAC_SCALE / 10000u)
    };    uint32_t t;      /* 放大后的精确整数 */
    uint32_t ip;     /* 整数部分 */
    uint32_t fp;     /* 小数部分 */
    uint32_t sgn;
    uint32_t i;
    float abs_v;
    char digits[10];
    int cnt;

    /* NaN 判定: value != value；±Inf 与超量程由下面的饱和限幅一并处理，
     * 这样帧长有界，且送给 VOFA+ 的永远是可解析的十进制数 */
    if (value != value)
    {
        value = 0.0f;
    }
    if (value >= VOFA_ABS_MAX)
    {
        value = VOFA_ABS_MAX;
    }
    else if (value <= -VOFA_ABS_MAX)
    {
        value = -VOFA_ABS_MAX;
    }

    /* 注意 -0.0f 归一到 0.0000（IEEE 负零对示波器无意义，且少一个负号更易读） */
    if (value < 0.0f)
    {
        sgn = 1u;
        abs_v = -value;
    }
    else
    {
        sgn = 0u;
        abs_v = value;
    }

    t = vofa_scale_round(abs_v);
    ip = t / VOFA_FRAC_SCALE;
    fp = t % VOFA_FRAC_SCALE;

    if (sgn != 0u && p < end)
    {
        *p++ = '-';
    }

    /* 整数部分：低位先算出，再倒序输出；ip==0 时仍输出一位 '0' */
    cnt = 0;
    do
    {
        digits[cnt++] = (char)('0' + (int)(ip % 10u));
        ip /= 10u;
    } while (ip != 0u && cnt < (int)sizeof(digits));

    while (cnt > 0 && p < end)
    {
        *p++ = digits[--cnt];
    }

    if (p < end)
    {
        *p++ = '.';
    }

    /* 小数部分：固定 VOFA_FRAC_DIGITS 位，高位不足补零 */
    for (i = 0u; i < (uint32_t)VOFA_FRAC_DIGITS && p < end; i++)
    {
        *p++ = (char)('0' + (int)((fp / frac_unit[i]) % 10u));
    }

    return p;
}

/* ===========================================================================
 * 对外接口
 * ========================================================================= */

/**
 * @brief  保留 stdout 重定向。
 * @note   现在没有任何地方调用 printf，因此链接器不会再拉入 printfa 及软件双精度
 *         例程；保留它只是为了工程里若再出现单字符调试输出仍能走同一个串口。
 */
int fputc(int ch, FILE *f)
{
    (void)f;
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, VOFA_TX_TIMEOUT_MS);
    return ch;
}

void vofa_init(void)
{
    /* 启动中断接收，接收完成后进入回调（接收部分逻辑保持原样，未改动） */
    HAL_UART_Receive_IT(&huart2, vofa_rx_buffer, 16);
}

void vofa_send(float x1, float x2, float x3)
{
    char *p = vofa_line_buf;
    /* 预留 2 字节给 "\r\n"；越过 end 后不再写入数据字符，保证不越界 */
    const char *end = vofa_line_buf + (VOFA_LINE_MAX_LEN - 1) - 2;

    /* 上一次发送还没结束就直接丢弃本帧：绝不在控制任务里排队干等 */
    if (huart2.gState != HAL_UART_STATE_READY)
    {
        return;
    }

    p = vofa_put_float(p, end, x1);
    if (p < end)
    {
        *p++ = ',';
    }
    p = vofa_put_float(p, end, x2);
    if (p < end)
    {
        *p++ = ',';
    }
    p = vofa_put_float(p, end, x3);

    /* FireWater 以 \r\n 作为一帧结束 */
    if (p < vofa_line_buf + (VOFA_LINE_MAX_LEN - 1))
    {
        *p++ = '\r';
    }
    if (p < vofa_line_buf + (VOFA_LINE_MAX_LEN - 1))
    {
        *p++ = '\n';
    }

    (void)HAL_UART_Transmit(&huart2, (uint8_t *)vofa_line_buf,
                            (uint16_t)(p - vofa_line_buf), VOFA_TX_TIMEOUT_MS);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        /* 必须再次开启中断接收，实现持续接收 */
        HAL_UART_Receive_IT(&huart2, vofa_rx_buffer, 16);
    }
}
