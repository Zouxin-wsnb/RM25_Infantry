#ifndef GIMBAL_STATE_SEARCH_HPP
#define GIMBAL_STATE_SEARCH_HPP

#include "StateMachine.hpp"

// #include "bsp_USB.h"

#include "GMMotorHandler.hpp"
#include "Dr16.hpp"
#include "GM6020.hpp"
#include "GM3508.hpp"
#include "GM2006.hpp"

#include "IMU_Config.hpp"
#ifdef IMU_LIB_EKF_
#include "AHRS-EKF.hpp"
#elif defined(IMU_LIB_MOTION_FX_)
#include "AHRS-MFX.hpp"
#elif defined(IMU_LIB_ICM42688_)
#include "AHRS-ICM.hpp"
#else
#include "AHRS-MFX.hpp" // 默认实现
#endif

/**
 * @class GimbalStateSearch
 * @brief 云台搜索状态机，用于哨兵自瞄？
 * @todo 未完成，可能需要等待视觉组的接口。
 */
class GimbalStateSearch : public State
{
public:
    /**
     * @brief 构造函数。
     */
    GimbalStateSearch(GM6020 *YawMotor,
                      GM6020 *PitchMotor) : YawMotor(YawMotor),
                                            PitchMotor(PitchMotor) {};

    /**
     * @brief 析构函数。
     */
    ~GimbalStateSearch() {};

    /**
     * @brief 云台Yaw轴电机。
     */
    GM6020 *YawMotor;

    /**
     * @brief 云台Pitch轴电机。
     */
    GM6020 *PitchMotor;

    /**
     * @brief 初始化放松状态。
     */
    void Init() override;

    /**
     * @brief 进入放松状态。
     */
    void Enter() override;

    /**
     * @brief 执行放松状态。
     */
    void Execute() override;

    /**
     * @brief 退出放松状态。
     */
    void Exit() override;
};

#endif
