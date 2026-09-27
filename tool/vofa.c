#include "vofa.h"
#include <stdio.h>
#include <stdint.h>
#include "usart.h"

uint8_t vofa_tx_buffer[16];
uint8_t vofa_rx_buffer[16];

int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart2, (uint8_t *)&ch, 1, 0x1f);
    return ch;
}

void vofa_init(void)
{
    // 启动中断接收，接收1个字节后进入回调
    HAL_UART_Receive_IT(&huart2, vofa_rx_buffer, 16);
}

void vofa_send(float x1, float x2, float x3)
{
    printf("%f,%f,%f\n", x1, x2, x3);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if(huart->Instance == USART2)
    {
        // 必须再次开启中断接收，实现持续接收
        HAL_UART_Receive_IT(&huart2, vofa_rx_buffer, 16);
    }
}
