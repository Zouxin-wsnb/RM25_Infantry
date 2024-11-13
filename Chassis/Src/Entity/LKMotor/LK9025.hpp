#ifndef LK9025_HPP
#define LK9025_HPP

#include "main.h"
#include "LKMotor.hpp"

/**
 * @class LK9025
 * @brief LK9025电机的控制类，继承自LKMotor类。
 *
 * 这个类实现了LK9025电机的控制方法，包括输出设置等。
 * LK9025电机通常用于精确的运动控制和高负载应用。
 */
class LK9025 : public LKMotor
{
public:
    /**
     * @brief 构造函数，初始化LK9025电机控制类。
     * 
     * 初始化电机的控制模式、各种设定值和PID控制器。
     */
    LK9025();

    /**
     * @brief 构造函数，初始化LK9025电机控制类。
     */
    ~LK9025();
    
    /**
     * @brief 实现电机输出设置。
     * 根据当前的控制模式和PID反馈调整电机输出。
     * 必须在派生类中具体实现此方法以适应具体电机。
     */
    void setOutput() override;
};

#endif // LK9025_HPP
