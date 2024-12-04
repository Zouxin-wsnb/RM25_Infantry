#ifndef DR16_HPP
#define DR16_HPP

#include "main.h"

#include "bsp_usart.hpp"
#include "remoteControl.hpp"

#include "Monitor.hpp"

class Dr16
{
public:
    /**
     * @enum PC_KEY_TYPE
     * @brief 定义电脑按键的类型。
     */
    typedef enum
    {
        PC_KEY_W = ((uint16_t)1 << 0),     ///< 按键W
        PC_KEY_S = ((uint16_t)1 << 1),     ///< 按键S
        PC_KEY_A = ((uint16_t)1 << 2),     ///< 按键A
        PC_KEY_D = ((uint16_t)1 << 3),     ///< 按键D
        PC_KEY_SHIFT = ((uint16_t)1 << 4), ///< 按键Shift
        PC_KEY_CTRL = ((uint16_t)1 << 5),  ///< 按键Ctrl
        PC_KEY_Q = ((uint16_t)1 << 6),     ///< 按键Q
        PC_KEY_E = ((uint16_t)1 << 7),     ///< 按键E
        PC_KEY_R = ((uint16_t)1 << 8),     ///< 按键R
        PC_KEY_F = ((uint16_t)1 << 9),     ///< 按键F
        PC_KEY_G = ((uint16_t)1 << 10),    ///< 按键G
        PC_KEY_Z = ((uint16_t)1 << 11),    ///< 按键Z
        PC_KEY_X = ((uint16_t)1 << 12),    ///< 按键X
        PC_KEY_C = ((uint16_t)1 << 13),    ///< 按键C
        PC_KEY_V = ((uint16_t)1 << 14),    ///< 按键V
        PC_KEY_B = ((uint16_t)1 << 15),    ///< 按键B
    } PC_KEY_TYPE;

    /**
     * @enum PC_KEY_STATE
     * @brief 定义电脑按键的状态类型。
     */
    typedef enum
    {
        PC_KEY_DOWN, ///< 按键保持按下
        PC_KEY_UP,   ///< 按键保持松开
        PC_KEY_FALL, ///< 按键下降沿
        PC_KEY_RISE  ///< 按键上升沿
    } PC_KEY_STATE;

    /**
     * @enum RC_SWITCH_TYPE
     * @brief 定义遥控器开关的类型。
     * @note 该枚举类型定义了遥控器上的两个开关，以及这两个开关的状态变化。
     * @param LEFT_SWITCH 左侧开关状态
     * @param RIGHT_SWITCH 右侧开关状态
     * @param LEFT_SWITCH_CHANGE 左侧开关状态变化
     * @param RIGHT_SWITCH_CHANGE 右侧开关状态变化
     */
    typedef enum
    {
        LEFT_SWITCH = 0,
        RIGHT_SWITCH,
        LEFT_SWITCH_CHANGE,
        RIGHT_SWITCH_CHANGE
    } RC_SWITCH_TYPE;

    /**
     * @enum RC_SWITCH_STATE
     * @brief 定义遥控器开关的状态类型。
     * @param RC_SW_UP 开关上
     * @param RC_SW_MID 开关中
     * @param RC_SW_DOWN 开关下
     * @param RC_SWITCH_M2D 从中到下
     * @param RC_SWITCH_M2U 从中到上
     * @param RC_SWITCH_D2M 从下到中
     * @param RC_SWITCH_U2M 从上到中
     */
    typedef enum
    {
        RC_SW_UP = 1,   ///< 开关上
        RC_SW_MID = 3,  ///< 开关中
        RC_SW_DOWN = 2, ///< 开关下

        RC_SWITCH_M2D = 4, ///< 从中到下
        RC_SWITCH_M2U = 5, ///< 从中到上
        RC_SWITCH_D2M = 6, ///< 从下到中
        RC_SWITCH_U2M = 7  ///< 从上到中
    } RC_SWITCH_STATE;

    Dr16();

    ~Dr16();

    /**
     * @brief 初始化遥控器。在main函数中调用。
     */
    void Dr16_Init();

    /**
     * @brief 映射遥控器的摇杆值。
     * 將遥控器的摇杆值映射到-1到1之间。
     */
    float MapAvix(int16_t ch);

    /**
     * @brief 更新遥控器摇杆和开关状态。
     */
    void UpdateRcStatus();

    /**
     * @brief 更新电脑按键状态。
     */
    void UpdateKeyStatus();

    /**
     * @brief 获取鼠标X轴的值。
     * @return 鼠标X轴的值。
     */
    float GetMouseX();

    /**
     * @brief 获取鼠标Y轴的值。
     * @return 鼠标Y轴的值。
     */
    float GetMouseY();

    /**
     * @brief 获取右侧摇杆X轴的值。
     * @return 右侧摇杆X轴的值。
     */
    float GetRightX();

    /**
     * @brief 获取右侧摇杆Y轴的值。
     * @return 右侧摇杆Y轴的值。
     */
    float GetRightY();

    /**
     * @brief 获取左侧摇杆X轴的值。
     * @return 左侧摇杆X轴的值。
     */
    float GetLeftX();

    /**
     * @brief 获取左侧摇杆Y轴的值。
     * @return 左侧摇杆Y轴的值。
     */
    float GetLeftY();

    /**
     * @brief 查询电脑按键的状态。
     * @param key 电脑按键的类型。
     * @return 电脑按键的状态。
     */
    PC_KEY_STATE QueryPcKeyStatus(PC_KEY_TYPE key);

    /**
     * @brief 查询遥控器开关的状态。
     * @param sw 遥控器开关的类型。
     * @return 遥控器开关的状态。
     */
    RC_SWITCH_STATE QuerySwStatus(RC_SWITCH_TYPE sw);

    /**
     * @brief 更新遥控器状态。
     * @note 该函数在process_remote_control_data中调用。
     */
    void Update();

    /**
     * @brief 获取遥控器类的实例。
     */
    static Dr16 *Instance() ///< 遥控器类的实例。
    {
        static Dr16 instance;
        return &instance;
    }

private:
    RC_ctrl_t *rc_raw; ///< 遥控器原始数据指针。

    RC_SWITCH_STATE Left_CurrentSw;  ///< 左侧开关当前状态。
    RC_SWITCH_STATE Left_PreviousSw; ///< 左侧开关上一次状态。
    RC_SWITCH_STATE Left_SwChange;   ///< 左侧开关状态变化。

    RC_SWITCH_STATE Right_CurrentSw;  ///< 右侧开关当前状态。
    RC_SWITCH_STATE Right_PreviousSw; ///< 右侧开关上一次状态。
    RC_SWITCH_STATE Right_SwChange;   ///< 右侧开关状态变化。

    uint16_t CurrentKeyState;  ///< 当前键盘按键状态。
    uint16_t PreviousKeyState; ///< 上一次键盘按键状态。
};

#endif // DR16_HPP
