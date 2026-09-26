#include "bsp_can.h"

/// @brief CAN1滤波器初始化
/// @param
void Can1_Filter_Init(void)
{
    CAN_FilterTypeDef can_filter_st;
    can_filter_st.FilterActivation = ENABLE;
    can_filter_st.FilterMode = CAN_FILTERMODE_IDMASK;
    can_filter_st.FilterScale = CAN_FILTERSCALE_32BIT;
    can_filter_st.FilterIdHigh = 0x0000;
    can_filter_st.FilterIdLow = 0x0000;
    can_filter_st.FilterMaskIdHigh = 0x0000;
    can_filter_st.FilterMaskIdLow = 0x0000;
    can_filter_st.FilterBank = 0;
    can_filter_st.FilterFIFOAssignment = CAN_RX_FIFO0;
    HAL_CAN_ConfigFilter(&hcan1, &can_filter_st);
    HAL_CAN_Start(&hcan1);
    HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING);
}

/* 在 STM32 CAN 接收中断回调中挂载处理函数 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    CAN_RxHeaderTypeDef rx_header;
    uint8_t rx_data[8];
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &rx_header, rx_data) == HAL_OK)
    {
        uint8_t bus = (hcan->Instance == CAN1) ? 1 : 2;
        // 调用管理器处理函数，这会实时刷新结构体中的 out_angle 等数据
        DamiaoMotorManager_RxHandler(bus, rx_header.StdId, rx_data);
    }
}

/**
 * @brief  底层硬件发送接口回调 (需自行实现)
 * @param  bus_id  CAN总线 (1或2)
 * @param  std_id  需发送的标准帧 ID
 * @param  data    数据指针
 * @param  len     数据长度 (固定为8)
 */
void User_CAN_Transmit(uint8_t bus_id, uint32_t std_id, uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef tx_hdr = {0};
    tx_hdr.StdId = std_id;
    tx_hdr.IDE = CAN_ID_STD;
    tx_hdr.RTR = CAN_RTR_DATA;
    tx_hdr.DLC = len;

    uint32_t mailbox;
    // CAN_HandleTypeDef *hcan = (bus_id == 1) ? &hcan1 : &hcan2;
    CAN_HandleTypeDef *hcan = (bus_id == 1) ? &hcan1 : &hcan1; // 由于目前仅使用到CAN1，所以暂时将CAN2也指向CAN1，避免未定义的hcan2问题
    HAL_CAN_AddTxMessage(hcan, &tx_hdr, data, &mailbox);
}
