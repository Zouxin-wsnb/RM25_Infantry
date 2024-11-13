#ifndef __BSP_IIC_HPP
#define __BSP_IIC_HPP

#include "main.h"

/**
 * @brief  IIC工作模式
 * @param IIC_MODE_BLOCK: 阻塞模式
 * @param IIC_MODE_INT: 中断模式
 * @param IIC_MODE_DMA: DMA模式
 */
typedef enum
{
    IIC_MODE_BLOCK = 0,
    IIC_MODE_IT,
    IIC_MODE_DMA,
} IIC_WORK_MODE;

/**
 * @brief  IIC 初始化
 * 只是形式上封装
 */
void I2C_Init(void);

/**
 * @brief  IIC 写数据
 * @param  hi2c: I2C句柄
 * @param  dev_addr: 设备地址
 * @param  reg_addr: 寄存器地址
 * @param  pData: 数据
 * @param  len: 数据长度
 * @todo 加入阻塞、中断、DMA模式
 */
void I2C_WriteData(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t reg_addr, uint16_t mem_addsize, uint8_t *pData, uint16_t len);

/**
 * @brief  IIC 读数据
 * @param  hi2c: I2C句柄
 * @param  dev_addr: 设备地址
 * @param  reg_addr: 寄存器地址
 * @param  pData: 数据
 * @param  len: 数据长度
 * @todo 加入阻塞、中断、DMA模式
 */
void I2C_ReadData(I2C_HandleTypeDef *hi2c, uint16_t dev_addr, uint16_t reg_addr, uint16_t mem_addsize, uint8_t *pData, uint16_t len);

#endif
