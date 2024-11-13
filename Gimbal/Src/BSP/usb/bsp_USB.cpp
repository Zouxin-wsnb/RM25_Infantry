/**
 * @file bsp_usb.c
 * @author your name (you@domain.com)
 * @brief usb是单例bsp,因此不保存实例
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */

#include "bsp_usb.hpp"
#include "bsp_dwt.hpp"

static uint8_t *bsp_usb_rx_buffer; // 接收到的数据会被放在这里,buffer size为2048
// 注意usb单个数据包(Full speed模式下)最大为64byte,超出可能会出现丢包情况

void USB_Init()
{
    // usb的软件复位(模拟拔插)在usbd_conf.c中的HAL_PCD_MspInit()中
    bsp_usb_rx_buffer = CDCInitRxbufferNcallback(); // 获取接收数据指针
}

void USB_Transmit(uint8_t *buffer, uint16_t len)
{
    CDC_Transmit_FS(buffer, len); // 发送
}

void USB_Receive(uint8_t *buffer, uint16_t len)
{
    memcpy(buffer, bsp_usb_rx_buffer, len); // 读取
	memset(bsp_usb_rx_buffer, 0, len); // 清空
}
