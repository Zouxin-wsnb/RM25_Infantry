#ifndef REFEREE_HPP
#define REFEREE_HPP

#include "main.h"
#include "bsp_usart.hpp"
#include "Monitor.hpp"
#include "crc.hpp"

#define JS_FRAME_HEADER_LEN 5
#define JS_FRAME_TAIL_LEN 2
#define JS_FRAME_HEADER_SOF 0xA5
#define JS_MAX_LEN 127

enum JudgeID
{
    GameStatus = 0x0001,              // 比赛状态，1Hz
    GameResult = 0x0002,              // 比赛结果，比赛结束后发送
    RobotHP = 0x0003,                 // 机器人血量。3Hz
    EventData = 0x0101,               // 场地事件数据。3Hz
    SupplyProjectileAction = 0x0102,  // 补给站动作标识数据，补给站弹丸释放时发送
    RefereeWarning = 0x0104,          // 裁判警告信息，己方判罚/判负时发送，其余事件1Hz
    DartInfo = 0x0105,                // 飞镖发射数据，1Hz
    GameRobotStatus = 0x0201,         // 机器人性能体系状态数据，1Hz
    PowerHeatData = 0x0202,           // 实时底盘功率和枪口热量数据 50Hz
    GameRobotPos = 0x0203,            // 机器人位置数据，1Hz
    Buff = 0x0204,                    // 机器人增益数据，3Hz
    AirSupportData = 0x0205,          // 空中支援时间数据 1Hz
    RobotHurt = 0x0206,               // 伤害状态数据，伤害发生后发送
    ShootData = 0x0207,               // 实时射击数据，弹丸发射后发送
    ProjectileAllowance = 0x0208,     // 允许发弹量 10Hz
    RfidStatus = 0x0209,              // 机器人 RFID 模块状态，3Hz
    DartClientCmd = 0x020A,           // 飞镖选手端指令数据，3Hz，只发给飞镖
    GroundRobotPosition = 0x020B,     // 地面机器人位置数据，1Hz，只发给哨兵
    RadarMarkData = 0x020C,           // 雷达标记进度数据，1Hz，只发给雷达
    SentryInfo = 0x020D,              // 哨兵自主决策信息同步，1Hz，只发给哨兵
    RadarInfo = 0x020E,               // 雷达自主决策信息同步，固定以1Hz
    RoboInteractData = 0x0301,        // 机器人交互数据，发送方触发发，频率上限为 30Hz
    CustomController2Robot = 0x0302,  // 自定义控制器与机器人交互数据，发送方触发发送，频率上限30Hz
    MapData = 0x0303,                 // 选手端小地图交互数据，选手端触发发送
    keyMouseData = 0x0304,            // 键鼠数据，30Hz
    RadarReceivedData = 0x0305,       // 选手端小地图接收雷达数据，频率上限为10Hz
    CustomController2Player = 0x0306, // 自定义控制器与选手端交互数据，发送方触发发送，频率上限30Hz
    SentryReceivedData = 0x0307,      // 选手端小地图接收哨兵数据，频率上限1Hz
    RobotReceivedData = 0x0308,       // 选手端小地图接收机器人数据，频率上限3Hz
};

struct FrameHeader
{
    uint8_t SOF;         // 帧头，固定为 0xA5
    uint16_t dataLength; // 数据长度
    uint8_t seq;         // 包序号
    uint8_t crc8;        // 帧头 CRC8 校验
} __attribute__((packed));

/**
 * @struct GameStatus_t
 * @brief 比赛状态数据结构体，0x0001
 */
struct GameStatus_t
{
    uint8_t Game_type : 4;      // 比赛类型
    uint8_t Game_progress : 4;  // 当前比赛阶段
    uint16_t Stage_remain_time; // 当前阶段剩余时间
    uint64_t SyncTimeStamp;     // UNIX 时间，当机器人正确连接到裁判系统的 NTP 服务器后生效
} __attribute__((packed));

/**
 * @struct GameResult_t
 * @brief 比赛结果数据结构体，0x0002
 */
struct GameResult_t
{
    uint8_t winner; // 比赛结果，0-平局，1-红方胜，2-蓝方胜
} __attribute__((packed));

enum RobotId
{
    RedHero = 1,
    RedEngineer = 2,
    RedInfantry_3 = 3,
    RedInfantry_4 = 4,
    RedInfantry_5 = 5,
    RedDrone = 6,
    RedSentry = 7,
    RedDart = 8,
    RedRadar = 9,

    BlueHero = 101,
    BlueEngineer = 102,
    BlueInfantry_3 = 103,
    BlueInfantry_4 = 104,
    BlueInfantry_5 = 105,
    BlueDrone = 106,
    BlueSentry = 107,
    BlueDart = 108,
    BlueRadar = 109,

    ClientRedHero = 0x0101,
    ClientRedEngineer = 0x0102,
    ClientRedInfantry_3 = 0x0103,
    ClientRedInfantry_4 = 0x0104,
    ClientRedInfantry_5 = 0x0105,
    ClientRedDrone = 0x0106,
    ClientBlueHero = 0x0165,
    ClientBlueEngineer = 0x0166,
    ClientBlueInfantry_3 = 0x0167,
    ClientBlueInfantry_4 = 0x0168,
    ClientBlueInfantry_5 = 0x0169,
    ClientBlueDrone = 0x016A,
};

/**
 * @struct RobotHP_t
 * @brief 机器人血量数据结构体，0x0003
 */
struct RobotHP_t
{
    uint16_t red_1_robot_HP;
    uint16_t red_2_robot_HP;
    uint16_t red_3_robot_HP;
    uint16_t red_4_robot_HP;
    uint16_t red_5_robot_HP;
    uint16_t red_7_robot_HP;
    uint16_t red_outpost_HP;
    uint16_t red_base_HP;
    uint16_t blue_1_robot_HP;
    uint16_t blue_2_robot_HP;
    uint16_t blue_3_robot_HP;
    uint16_t blue_4_robot_HP;
    uint16_t blue_5_robot_HP;
    uint16_t blue_7_robot_HP;
    uint16_t blue_outpost_HP;
    uint16_t blue_base_HP;
} __attribute__((packed));

/**
 * @struct EventData_t
 * @brief 场地事件数据结构体
 * 0x0101
 */
struct EventData_t
{
    uint32_t event_data; // 事件类型
} __attribute__((packed));

/**
 * @struct SupplyProjectileAction_t
 * @brief 补给站动作标识数据结构体
 * 0x0102
 */
struct SupplyProjectileAction_t
{
    uint8_t reserved;
    uint8_t supply_robot_id;        // 补弹机器人 ID
    uint8_t supply_projectile_step; // 出弹口开闭状态
    uint8_t supply_projectile_num;  // 补弹数量
} __attribute__((packed));

/**
 * @struct RefereeWarning_t
 * @brief 裁判警告信息数据结构体
 * 0x0104
 */
struct RefereeWarning_T
{
    uint8_t level;              // 己方最后一次受到判罚的等级
    uint8_t offending_robot_id; // 己方最后一次受到判罚的违规机器人 ID。
    uint8_t count;              // 己方最后一次受到判罚的违规机器人对应判罚等级的违规次数。（开局默认为 0。）
} __attribute__((packed));

/**
 * @struct DartInfo_t
 * @brief 飞镖发射数据结构体
 * 0x0105
 */
struct DartInfo_t
{
    uint8_t dart_remaining_time; // 己方飞镖发射剩余时间，单位：秒
    uint16_t dart_info;
} __attribute__((packed));

/**
 * @struct GameRobotStatus_t
 * @brief 机器人性能体系状态数据结构体
 * 0x0201
 */
struct GameRobotStatus_t
{
    uint8_t robot_id;                            // 本机器人 ID
    uint8_t robot_level;                         // 机器人等级
    uint16_t current_HP;                         // 当前血量
    uint16_t maximum_HP;                         // 血量上限
    uint16_t shooter_barrel_cooling_value;       // 机器人枪口热量每秒冷却值
    uint16_t shooter_barrel_heat_limit;          // 机器人枪口热量上限
    uint16_t chassis_power_limit;                // 机器人底盘功率上限
    uint8_t power_management_gimbal_output : 1;  // bit 0：gimbal 口输出：0 为无输出，1 为 24V 输出
    uint8_t power_management_chassis_output : 1; // bit 1：chassis 口输出：0 为无输出，1 为 24V 输出
    uint8_t power_management_shooter_output : 1; // bit 2：shooter 口输出：0 为无输出，1 为 24V 输出
} __attribute__((packed));

/**
 * @struct PowerHeatData_t
 * @brief 实时底盘功率和枪口热量数据结构体
 * 0x0202
 */
struct PowerHeatData_t
{
    uint16_t chassis_volt;                // 电源管理模块的 chassis 口输出电压（单位：mV）
    uint16_t chassis_current;             // 电源管理模块的 chassis 口输出电流（单位：mA）
    float chassis_power;                  // 底盘功率（单位：W）
    uint16_t chassis_power_buffer;        // 缓冲能量（单位：J）
    uint16_t shoot_id1_17mm_cooling_heat; // 第 1 个 17mm 发射机构的枪口热量
    uint16_t shoot_id2_17mm_cooling_heat; // 第 2 个 17mm 发射机构的枪口热量
    uint16_t shoot_id1_42mm_cooling_heat; // 42mm 发射机构的枪口热量
} __attribute__((packed));

/**
 * @struct GameRobotPos_t
 * @brief 机器人位置数据结构体
 * 0x0203
 */
struct GameRobotPos_t
{
    float x;   // 本机器人位置 x 坐标，单位：m
    float y;   // 本机器人位置 y 坐标，单位：m
    float yaw; // 本机器人测速模块的朝向，单位：度。正北为 0 度
} __attribute__((packed));

/**
 * @struct Buff_t
 * @brief 机器人增益数据结构体
 * 0x0204
 */
struct Buff_t
{
    uint8_t recovery_buff;      // 机器人回血增益（百分比，值为 10 表示每秒恢复血量上限的 10%）
    uint8_t cooling_buff;       // 机器人枪口冷却倍率（直接值，值为 5 表示 5 倍冷却）
    uint8_t defence_buff;       // 机器人防御增益（百分比，值为 50 表示 50%防御增益）
    uint8_t vulnerability_buff; // 机器人负防御增益（百分比，值为 30 表示-30%防御增益）
    uint16_t attack_buff;       // 机器人攻击增益（百分比，值为 50 表示 50%攻击增益）
} __attribute__((packed));

/**
 * @struct AirSupportData_t
 * @brief 空中支援时间数据结构体
 * 0x0205
 */
struct AirSupportData_t
{
    uint8_t airforce_status; // 空中机器人状态（0 为正在冷却，1 为冷却完毕，2 为正在空中支援）
    uint8_t time_remain;     // 此状态的剩余时间（单位为：秒，向下取整，即冷却时间剩余 1.9 秒时，此值为 1）若冷却时间为 0，但未呼叫空中支援，则该值为 0
} __attribute__((packed));

/**
 * @struct RobotHurt_t
 * @brief 伤害状态数据结构体
 * 0x0206
 */
struct RobotHurt_t
{
    uint8_t armor_id : 4;            // bit 0-3：当扣血原因为装甲模块被弹丸攻击、受撞击、离线或测速模块离线时，该 4 bit 组成的数值为装甲模块或测速模块的 ID 编号；当其他原因导致扣血时，该数值为 0
    uint8_t HP_deduction_reason : 4; // bit 4-7：血量变化类型，0：装甲模块被弹丸攻击导致扣血，1：裁判系统重要模块离线导致扣血，2：射击初速度超限导致扣血，3：枪口热量超限导致扣血，4：底盘功率超限导致扣血，5：装甲模块受到撞击导致扣血
} __attribute__((packed));

/**
 * @struct ShootData_t
 * @brief 实时射击数据结构体
 * 0x0207
 */
struct ShootData_t
{
    uint8_t bullet_type; // 弹丸类型。1：17mm 弹丸，2：42mm 弹丸
    uint8_t shooter_id;  // 发射机构 ID，1：第一个 17mm 发射机构，2：第二个 17mm 发射机构，3：42mm 发射机构
    uint8_t bullet_freq; // 弹丸射频，单位：Hz
    float bullet_speed;  // 弹丸射速，单位：m/s
} __attribute__((packed));

/**
 * @struct ProjectileAllowance_t
 * @brief 允许发弹量数据结构体
 * 0x0208
 */
struct BulletRemaining_t
{
    uint16_t bullet_remaining_num_17mm; // 17mm 弹丸剩余发射数量
    uint16_t bullet_remaining_num_42mm; // 42mm 弹丸剩余发射数量
    uint16_t coin_remaining_num;        // 剩余补弹币数量
} __attribute__((packed));

/**
 * @struct RfidStatus_t
 * @brief 机器人 RFID 模块状态数据结构体
 * 0x0209
 */
struct RfidStatus_t
{
    uint32_t rfid_status;
} __attribute__((packed));

/**
 * @struct DartClientCmd_t
 * @brief 飞镖选手端指令数据结构体
 * 0x020A
 */
struct DartClientCmd_t
{
    uint8_t dart_launch_opening_status; // 飞镖发射口开闭状态，1：关闭，2：正在开启或者关闭，0：已经开启
    uint8_t reserved;                   // 保留
    uint16_t target_change_time;        // 切换击打目标时的比赛剩余时间，单位：秒，无/未切换动作，默认为 0。
    uint16_t latest_launch_cmd_time;    // 最后一次操作手确定发射指令时的比赛剩余时间，单位：秒，初始值为 0
} __attribute__((packed));

/**
 * @struct GroundRobotPosition_t
 * @brief 地面机器人位置数据结构体
 * 0x020B
 */
struct GroundRobotPosition_t
{
    float hero_x;
    float hero_y;
    float engineer_x;
    float engineer_y;
    float standard_3_x;
    float standard_3_y;
    float standard_4_x;
    float standard_4_y;
    float standard_5_x;
    float standard_5_y;
} __attribute__((packed));

/**
 * @struct RadarMarkData_t
 * @brief 雷达标记进度数据结构体
 * 0x020C
 */
struct RadarMarkData_t
{
    uint8_t mark_hero_progress;       // 对方英雄机器人被标记进度：0-120
    uint8_t mark_engineer_progress;   // 对方工程机器人被标记进度：0-120
    uint8_t mark_standard_3_progress; // 对方步兵 3 机器人被标记进度：0-120
    uint8_t mark_standard_4_progress; // 对方步兵 4 机器人被标记进度：0-120
    uint8_t mark_standard_5_progress; // 对方步兵 5 机器人被标记进度：0-120
    uint8_t mark_sentry_progress;     // 对方哨兵机器人被标记进度：0-120
} __attribute__((packed));

/**
 * @struct SentryInfo_t
 * @brief 哨兵自主决策信息同步数据结构体
 * 0x020D
 */
struct SentryInfo_t
{
    uint32_t sentry_info;
} __attribute__((packed));

/**
 * @struct RadarInfo_t
 * @brief 雷达自主决策信息同步数据结构体
 * 0x020E
 */
struct RadarInfo_t
{
    uint8_t radar_info;
} __attribute__((packed));

/**
 * @struct RoboInteractData_t
 * @brief 机器人交互数据结构体
 * 0x0301
 */
struct RoboInteractData_t
{
    uint16_t data_cmd_id; // 子内容 ID 需为开放的子内容 ID，0x0200~0x02FF
    uint16_t sender_id;   // 发送者 ID 需与自身 ID 匹配
    uint16_t receiver_id; // 接收者 ID
} __attribute__((packed));

/**
 * @struct DeleteGraphic_t
 * @brief 删除图形数据结构体
 * 0x0100
 */
struct DeleteGraphic_t
{
    uint8_t delete_type; // 删除类型，0：空操作，1：删除图层，2：删除所有
    uint8_t layer;       // 图层数：0~9
} __attribute__((packed));

class Referee
{
public:
    Referee();
    ~Referee();
    GameResult_t GameResult;
    GameRobotStatus_t GameRobotStatus;
    PowerHeatData_t PowerHeatData;

    void ProcessData(uint8_t *pData, uint16_t Size);
    PowerHeatData_t GetPowerHeatData();

    static Referee *Instance()
    {
        static Referee referee;
        return &referee;
    }
};
#endif
