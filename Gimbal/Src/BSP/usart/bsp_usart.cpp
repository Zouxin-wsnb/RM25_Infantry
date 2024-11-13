#include "bsp_usart.hpp"
#include "remoteControl.hpp"
#include "stm32f4xx_it.h"
#include "Monitor.hpp"
#include "Dr16.hpp"
/*------------全局变量------------*/
extern UART_HandleTypeDef huart3;
extern DMA_HandleTypeDef hdma_usart3_rx;

extern UART_HandleTypeDef huart6;
extern DMA_HandleTypeDef hdma_usart6_rx;
extern DMA_HandleTypeDef hdma_usart6_tx;

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_rx;
extern DMA_HandleTypeDef hdma_usart1_tx;

#define RXBUFFERSIZE 512
#define TXBUFFERSIZE 512

static uint8_t UART1RxBuffer[RXBUFFERSIZE];
static uint8_t UART6RxBuffer[RXBUFFERSIZE];

static uint8_t TxBuffer[TXBUFFERSIZE];

void USART_Init(void)
{
  USART1_Init();
  USART6_Init();
}

void USART6_Init()
{
  __HAL_DMA_DISABLE_IT(&hdma_usart6_rx, DMA_IT_HT); // 禁用半传输中断
  __HAL_DMA_ENABLE_IT(&hdma_usart6_rx, DMA_IT_TC);  // 启用传输完成中断
  __HAL_DMA_DISABLE_IT(&hdma_usart6_tx, DMA_IT_HT); // 禁用半传输中断
  __HAL_DMA_ENABLE_IT(&hdma_usart6_tx, DMA_IT_TC);  // 启用传输完成中断

  // 开启DMA接收
  // HAL_UART_Receive_DMA(&huart6, aRxBuffer, 2);
  uint8_t message[] = "[INFO]USART6 Init Success!\r\n";
  USART_Transmit(&huart1, message, sizeof(message), USART_MODE_BLOCK);
};

void USART1_Init()
{
  __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT); // 禁用半传输中断
  __HAL_DMA_ENABLE_IT(&hdma_usart1_rx, DMA_IT_TC);  // 启用传输完成中断
  __HAL_DMA_DISABLE_IT(&hdma_usart1_tx, DMA_IT_HT); // 禁用半传输中断
  __HAL_DMA_ENABLE_IT(&hdma_usart1_tx, DMA_IT_TC);  // 启用传输完成中断

  // 开启DMA接收
  // HAL_UART_Receive_DMA(&huart6, aRxBuffer, 2);
  uint8_t message[] = "[INFO]USART1 Init Success!\r\n";
  USART_Transmit(&huart1, message, sizeof(message), USART_MODE_BLOCK);
};

void USART3_Init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
  // 使能DMA窗口接收
  SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);
  // 使能空闲中断
  __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);
  // 失效DMA
  __HAL_DMA_DISABLE(&hdma_usart3_rx);
  while (hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
  {
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
  }

  hdma_usart3_rx.Instance->PAR = (uint32_t)&(USART3->DR);

  // 内存缓冲区1
  hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
  // 内存缓冲区2
  hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
  // 数据长度
  hdma_usart3_rx.Instance->NDTR = dma_buf_num;

  // 使能双缓冲区
  SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);
  // 使能DMA
  __HAL_DMA_ENABLE(&hdma_usart3_rx);

  uint8_t message[] = "[INFO]USART3 Init Success!\r\n";
  USART_Transmit(&huart1, message, sizeof(message), USART_MODE_BLOCK);

  /*---------------------LL库配置----------------------*/
  // LL_USART_EnableDMAReq_RX(USART3); // 使能串口3的DMA接收请求

  // LL_USART_ClearFlag_IDLE(USART3); // 清除空闲中断标志位
  // LL_USART_EnableIT_IDLE(USART3);	 // 使能空闲中断
  // /* -------------- Configure DMA -----------------------------------------*/
  // LL_DMA_InitTypeDef DMA_InitStructure;
  // LL_DMA_DeInit(DMA1, LL_DMA_STREAM_1);

  // DMA_InitStructure.Channel = LL_DMA_CHANNEL_4;
  // DMA_InitStructure.PeriphOrM2MSrcAddress = (uint32_t) & (USART3->DR);
  // DMA_InitStructure.MemoryOrM2MDstAddress = (uint32_t)rx1_buf;
  // DMA_InitStructure.Direction = LL_DMA_DIRECTION_PERIPH_TO_MEMORY;
  // DMA_InitStructure.NbData = dma_buf_num;
  // DMA_InitStructure.PeriphOrM2MSrcIncMode = LL_DMA_PERIPH_NOINCREMENT;
  // DMA_InitStructure.MemoryOrM2MDstIncMode = LL_DMA_MEMORY_INCREMENT;
  // DMA_InitStructure.PeriphOrM2MSrcDataSize = LL_DMA_PDATAALIGN_BYTE;
  // DMA_InitStructure.MemoryOrM2MDstDataSize = LL_DMA_MDATAALIGN_BYTE;
  // DMA_InitStructure.Mode = LL_DMA_MODE_CIRCULAR;
  // DMA_InitStructure.Priority = LL_DMA_PRIORITY_VERYHIGH;
  // DMA_InitStructure.FIFOMode = LL_DMA_FIFOMODE_DISABLE;
  // DMA_InitStructure.FIFOThreshold = LL_DMA_FIFOTHRESHOLD_1_4;
  // DMA_InitStructure.MemBurst = LL_DMA_MBURST_SINGLE;
  // DMA_InitStructure.PeriphBurst = LL_DMA_PBURST_SINGLE;
  // LL_DMA_Init(DMA1, LL_DMA_STREAM_1, &DMA_InitStructure);				// 初始化DMA，尽管MX有相关初始化，但是有些设置需要重新设置
  // LL_DMA_SetMemory1Address(DMA1, LL_DMA_STREAM_1, (uint32_t)rx2_buf); // 设置DMA的第一个缓冲区地址
  // LL_DMA_EnableDoubleBufferMode(DMA1, LL_DMA_STREAM_1);				// 使能DMA的双缓冲模式
  // LL_DMA_DisableStream(DMA1, LL_DMA_STREAM_1);
  // LL_DMA_EnableStream(DMA1, LL_DMA_STREAM_1); // 使能DMA
}

void USART_Transmit(UART_HandleTypeDef *huart, uint8_t *pData, uint16_t Size, enum USART_Mode mode)
{
  memcpy(TxBuffer, pData, Size);
  switch (mode)
  {
  case USART_MODE_BLOCK:
    HAL_UART_Transmit(huart, TxBuffer, Size, 100);
    break;
  case USART_MODE_DMA:
    HAL_UART_Transmit_DMA(huart, TxBuffer, Size);
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
  if (huart->Instance == USART6)
    memcpy(pData, UART6RxBuffer, Size);
  else if (huart->Instance == USART1)
    memcpy(pData, UART1RxBuffer, Size);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART6)
  {
    HAL_UART_Receive_DMA(&huart6, UART6RxBuffer, 2);
  }
  if (huart->Instance == USART1)
  {
    HAL_UART_Receive_DMA(&huart1, UART1RxBuffer, 2);
  }
}

void USART3_IRQHandler(void)
{
  /* USER CODE BEGIN USART3_IRQn 0 */

  /* USER CODE END USART3_IRQn 0 */
  HAL_UART_IRQHandler(&huart3);
  /* USER CODE BEGIN USART3_IRQn 1 */
  process_remote_control_data();

  // 监控遥控器更新时间，用于检查遥控器是否正常
  Monitor::Instance()->RemoteController_Monitor();

  /* USER CODE END USART3_IRQn 1 */
}