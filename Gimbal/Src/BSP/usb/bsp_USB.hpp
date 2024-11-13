/**
 * @file bsp_usb.h
 * @author your name (you@domain.com)
 * @brief 提供对usb vpc(virtal com port)的操作接口,hid和msf考虑后续添加
 * @attention 这一版usb修改了usbd_cdc_if.c中的CDC_Receive_FS函数,若使用cube生成后会被覆盖.后续需要由usbcdciftemplate创建一套自己的模板
 * @version 0.1
 * @date 2023-02-09
 *
 * @copyright Copyright (c) 2023
 *
 */
#ifndef BSP_USB_HPP
#define BSP_USB_HPP

#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_conf.h"
#include "usbd_desc.h"
#include "usbd_cdc_if.h"

#include "string.h"

/**
 * @brief 初始化usb
 * 将usb的接收缓冲区指针指向usbd_cdc_if.c中的CDCInitRxbufferNcallback函数返回的指针
 */
void USB_Init();

/**
 * @brief 发送数据
 * @param buffer 发送的数据
 * @param len 数据长度
 */
void USB_Transmit(uint8_t *buffer, uint16_t len);

/**
 * @brief 接收数据
 * @param buffer 接收数据的缓冲区
 * @param len 数据长度
 */
void USB_Receive(uint8_t *buffer, uint16_t len);

#endif
