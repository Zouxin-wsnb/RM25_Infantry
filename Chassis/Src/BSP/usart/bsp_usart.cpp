#include "bsp_usart.hpp"
#include "stm32f4xx_it.h"
#include "Monitor.hpp"
#include "Referee.hpp"
/*------------全局变量------------*/
extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

#define RXBUFFERSIZE 256
#define TXBUFFERSIZE 256

uint8_t UART6RxBuffer[RXBUFFERSIZE] = {0};
uint8_t UART1RxBuffer[RXBUFFERSIZE] = {0};

uint8_t aTxBuffer[TXBUFFERSIZE];
// uint8_t UART1TxBuffer[TXBUFFERSIZE];

void USART_Init(void)
{
  USART1_Init();
  USART6_Init();
}

// USART6初始化，用于和裁判系统通信，收发数据
void USART6_Init()
{
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT); // 禁用半传输中断
  __HAL_DMA_ENABLE_IT(&hdma_usart6_rx, DMA_IT_TC);  // 启用传输完成中断

  __HAL_DMA_DISABLE_IT(&hdma_usart6_tx, DMA_IT_HT); // 禁用半传输中断
  __HAL_DMA_ENABLE_IT(&hdma_usart6_tx, DMA_IT_TC);  // 启用传输完成中断

  // 开启DMA接收
  HAL_UARTEx_ReceiveToIdle_DMA(&huart6, UART6RxBuffer, RXBUFFERSIZE); // 开启DMA空闲中断
  uint8_t message[] = "[INFO]USART6 Init Success!\r\n";
  USART_Transmit(&huart1, message, sizeof(message), USART_MODE_BLOCK);
};

void USART1_Init()
{
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); // 禁用半传输中断
  // __HAL_DMA_ENABLE_IT(&hdma_usart1_rx, DMA_IT_TC);  // 启用传输完成中断

  __HAL_DMA_DISABLE_IT(&hdma_usart1_tx, DMA_IT_HT); // 禁用半传输中断
  __HAL_DMA_ENABLE_IT(&hdma_usart1_tx, DMA_IT_TC);  // 启用传输完成中断

  // HAL_UARTEx_ReceiveToIdle_DMA(&huart1, UART1RxBuffer, RXBUFFERSIZE); // 开启DMA空闲中断
  uint8_t message[] = "[INFO]USART1 Init Success!\r\n";
  USART_Transmit(&huart1, message, sizeof(message), USART_MODE_BLOCK);
};

void USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, enum USART_Mode mode)
{
  memcpy(aTxBuffer, pData, Size);
  switch (mode)
  {
  case USART_MODE_BLOCK:
    HAL_UART_Transmit(huart, aTxBuffer, Size, 100);
    break;
  case USART_MODE_DMA:
    HAL_UART_Transmit_DMA(huart, aTxBuffer, Size);
    break;
  case USART_MODE_IT:
    break;
  default:
    break;
  }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
}

void USART_Receive(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size)
{
  if (Size > RXBUFFERSIZE)
  {
    return;
  }

  if (huart->Instance == USART6)
  {
    memcpy(pData, UART6RxBuffer, Size);
    memset(UART6RxBuffer, 0, Size);
  }
  else if (huart->Instance == USART1)
  {
    memcpy(pData, UART1RxBuffer, Size);
    memset(UART1RxBuffer, 0, Size);
  }
  else
  {
    return;
  }
}

/**
 * @brief 串口接受回调函数
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
}

/**
 * @brief 串口空闲中断回调函数
 */
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size)
 {
  if (huart->Instance == USART6)
  {
		// USART_Transmit(&huart6, UART6RxBuffer, Size, USART_MODE_DMA);
    Referee::Instance()->ProcessData(UART6RxBuffer, Size);
    HAL_UARTEx_ReceiveToIdle_DMA(&huart6, UART6RxBuffer, RXBUFFERSIZE);
  }
}