#include "bsp.hpp"

void bsp_init()
{
  CAN_Init();    // CAN初始化
  DWT_Init(168); // 168MHz，c板主频168MHz
  GPIO_Init();   // GPIO初始化，主要是将LED，SPI，IIC所需要的引脚初始化
  USB_Init();    // USB初始化
  USART_Init();
  Monitor::Instance()->Log_Messages(Monitor::INFO, (uint8_t *)("All BSP init Success!\r\n"));
}
