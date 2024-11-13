#ifndef BOARD_CONNECTIVITY_H
#define BOARD_CONNECTIVITY_H

#include <string.h> // for memcpy

#include "main.h"

#include "bsp_can.hpp"
#include "bsp_usart.hpp"

#include "Monitor.hpp"

// 对于云台控制板，发送的id范围是0xB1~0xB8，接收的id范围是0xC1~0xC8
// 对于底盘的底盘的C板，发送的id范围是0xC1~0xC8，接收的id范围是0xB1~0xB8
/**
 * @todo 以后做到只需要修改此处的宏定义即可完成板件通信发送接收ID的修改。
 */
#define BOARD_CONNECTIVITY_SEND_ID 0xB0    // 板件通信发送ID
#define BOARD_CONNECTIVITY_RECEIVE_ID 0xC0 // 板件通信接收ID

/**
 * @enum 板间通信的类型。
 * @param BOARD_CONNECTIVITY_CAN_1: CAN1通信。
 * @param BOARD_CONNECTIVITY_CAN_2: CAN2通信。
 * @param BOARD_CONNECTIVITY_USART: USART通信。实际尚未实现，可以用在裁判系统
 */
typedef enum
{
    BOARD_CONNECTIVITY_CAN_1 = 0,
    BOARD_CONNECTIVITY_CAN_2 = 1,
    BOARD_CONNECTIVITY_USART = 3,
} BoardConnectivityType;

/**
 * @struct 板间通信的数据结构。
 * @param type 板间通信的类型。
 * @param id 板间通信的ID，对于can通讯，即为帧id，对于串口通讯，即为帧头。
 * @param len 板间通信的数据长度。不可以超过8。
 * @param data 板间通信的数据。
 */
struct BoardMsg
{
    BoardConnectivityType type;
    uint16_t id;
    uint8_t len; // 数据长度，不可以超过8
    uint8_t data[8];
} __attribute__((packed));

/**
 * @class 板间通信类。用于和其他板子通信。
 */
class BoardConnectivity
{
public:
    BoardConnectivity();
    ~BoardConnectivity();

    /**
     * @brief 板间通信数据存储，数组索引对应板间通信的ID。
     * @note 目前最多存储8个板间通信数据，可以考虑是否需要更多。
     */
    BoardMsg BoardMemory_send[8];

    /**
     * @brief 板间通信数据存储，数组索引对应板间通信的ID。
     * @note 目前最多存储8个板间通信数据，可以考虑是否需要更多。
     */
    BoardMsg BoardMemory_receive[8];

    /**
     * @brief 初始化函数。
     * 将BoardMemory_send和BoardMemory_receive的长度清零。
     */
    void Init(void);

    /**
     * @brief 添加数据到板间通信的内存中。
     * @param msg 板间通信的数据结构体
     */
    void Add2Memory(BoardMsg &msg);

    /**
     * @brief 发送数据。
     * 将BoardMemory_send中的数据发送出去，在主循环中调用
     */
    void BoardConnectivity_Send();

    /**
     * @brief 获取数据。
     * @param id 需要的数据的ID
     * @param data 用于存储数据的数组
     * @param len 数据的长度
     */
    void GetBoardMemory(uint8_t id, uint8_t *data, uint8_t len);

    /**
     * @brief 清除数据。
     * @param id 需要清除的数据的ID
     */
    void ClearBoardMemory(uint8_t id);

    /**
     * @brief 获取BoardConnectivity的单例。
     */
    static BoardConnectivity *Instance(void)
    {
        static BoardConnectivity instance;
        return &instance;
    }
};

#endif // BOARD_CONNECTIVITY_H
