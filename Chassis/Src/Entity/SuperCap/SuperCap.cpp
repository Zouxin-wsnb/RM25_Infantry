#ifndef SUPER_CAP_HPP
#define SUPER_CAP_HPP

#include "BoardConnectivity.hpp"

struct SuperCapSetData
{
    BoardConnectivityType type;
    uint16_t id;
    uint8_t len; // 数据长度，不可以超过8

    uint8_t cap_state_set;
    uint8_t fly_extra_set;
    float power_limit_set;
    uint16_t reserve; // 保留位
} __attribute__((packed));

union SuperCapSetUnion
{
    SuperCapSetData capSetData;
    BoardMsg msg;
} __attribute__((packed));

struct SuperCapFdbData
{
    BoardConnectivityType type;
    uint16_t id;
    uint8_t len; // 数据长度，不可以超过8

    uint8_t cap_state_fdb;
    uint8_t cap_voltage_x5;
    int16_t input_power_x100;
    int16_t cap_power_x100;
    int16_t cap_inpower_x100;
} __attribute__((packed));

union SuperCapFdbUnion
{
    SuperCapFdbData capFdbData;
    BoardMsg msg;
} __attribute__((packed));

class SuperCap
{
public:
    SuperCapSetUnion SCSetUnion; ///< 用于发送给超级电容的数据
    SuperCapFdbUnion SCFdbUnion; ///< 用于接收超级电容的数据

    void Init();
    // 设置超级电容的状态
    void SetCapState(uint8_t state);
    void SetFlyExtra(uint8_t extra);
    void SetPowerLimit(float power);
    // 获取超级电容的状态
    void GetCapState();
    void GetCapVoltage();
    void GetInputPower();
    void GetCapPower();
    void GetCapInPower();

    void SendCapData();
};

#endif // SUPER_CAP_HPP
