#ifndef GIMBAL_PITCH_ADAPT_HPP
#define GIMBAL_PITCH_ADAPT_HPP

#include "StateMachine.hpp"

#include "GMMotorHandler.hpp"
#include "Dr16.hpp"
#include "GM6020.hpp"
#include "GM3508.hpp"
#include "GM2006.hpp"

class GimbalStatePitchAdapt : public State
{
public:
    /**
     * @brief 构造函数。
     * @param YawMotor 云台Yaw轴电机。
     */
    GimbalStatePitchAdapt(GM6020 *PitchMotor) : PitchMotor(PitchMotor) {};

    /**
     * @brief 析构函数。
     */
    ~GimbalStatePitchAdapt() {};

    /**
     * @brief 云台Pitch轴电机。
     */
    GM6020 *PitchMotor;

    /**
     * @brief Pitch轴电机的位置设定。
     * 单位为弧度，范围为[-∞, ∞]。
     * 最后会在execute函数中映射到[0, 2π]。
     * 并且还需要根据实际情况添加限幅
     */
    float PitchSet;

    /**
     * @brief 云台上限
     */
    float PitchUpperLimit;

    /**
     * @brief 云台下限
     */
    float PitchLowerLimit;

    bool AdaptFinished = false;

    void Init() override;
		
		void Enter() override;

    void Execute() override;

    void Exit() override;
};

#endif