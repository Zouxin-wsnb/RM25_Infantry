#ifndef LK8016_HPP
#define LK8016_HPP

#include "main.h"
#include "LKMotor.hpp"

/**
 * @class LK8016
 * @brief LK8016电机的控制类，继承自GMMotor类。
 *
 * 这个类实现了LK8016电机的控制方法，包括输出设置等。
 * LK8016电机通常用于精确的运动控制和高负载应用。
 */
class LK8016 : public LKMotor
{
public:
    /**
     * @brief 构造函数，初始化LK8016电机控制类。
     */
    LK8016();

    /**
     * @brief 构造函数，初始化LK8016电机控制类。
     */
    ~LK8016();
    
    /**
     * @brief 实现电机输出设置。
     * 根据当前的控制模式和PID反馈调整电机输出。
     * 必须在派生类中具体实现此方法以适应具体电机。
     */
    void setOutput() override;
};

#endif // LK8016_HPP
