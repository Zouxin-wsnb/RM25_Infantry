#ifndef CHASSIS_CONTROLLER_HPP
#define CHASSIS_CONTROLLER_HPP

#include "GMMotorHandler.hpp"

#include "StateMachine.hpp"

#include "ChassisStateRemoteControl.hpp"
#include "ChassisStateRelax.hpp"
#include "ChassisStateRotate.hpp"
#include "ChassisStateFPS.hpp"

#include "Dr16.hpp"
#include "Monitor.hpp"
#include "BoardConnectivity.hpp"

/**
 * @class ChassisController
 * @brief 底盘控制类，提供底盘的基本控制功能。
 * 在云台的程序中，这个类主要负责将处理好的数据发送给底盘控制板，地盘控制板会进一步处理。
 */
class ChassisController : public StateMachine
{
public:
    /*-----------------------------状态机-----------------------------*/

    /**
     * @brief 底盘遥控状态。
     */
    ChassisStateRemoteControl chassisStateRemoteControl;

    /**
     * @brief 底盘放松状态。
     */
    ChassisStateRelax chassisStateRelax;

    /**
     * @brief 底盘小陀螺。
     */
    ChassisStateRotate chassisStateRotate;

    /**
     * @brief 底盘跟随云台。
     */
    ChassisStateFPS chassisStateFPS;

    /**
     * @brief 构造函数。
     */
    ChassisController() {};
    /**
     * @brief 析构函数。
     */
    ~ChassisController() {};

    /**
     * @brief 初始化函数。
     * @note 将电机遥控数据加入到发送缓冲区。全部初始化为0。
     */
    void Init() override;

    /**
     * @brief 运行函数。
     * 根据遥控器的状态，选择不同的底盘状态。
     */
    void HandleInput() override;

    void Run() override;

    /**
     * @brief 获取底盘控制类的单例。
     * @return ChassisController* 返回底盘控制类的单例。
     */
    static ChassisController *Instance()
    {
        static ChassisController instance;
        return &instance;
    }
};

#endif // CHASSIS_CONTROLLER_HPP
