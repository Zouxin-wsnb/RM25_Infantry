#ifndef BOARD_CONNECTIVITY_H
#define BOARD_CONNECTIVITY_H

#include "main.h"

#include "bsp_can.hpp"
#include "bsp_usart.hpp"

#include "string.h" // for memcpy
#include "Monitor.hpp"

// 对于云台控制板，发送的id范围是0xB1~0xB8，接收的id范围是0xC1~0xC8
// 对于底盘的底盘的C板，发送的id范围是0xC1~0xC8，接收的id范围是0xB1~0xB8
#define BOARD_CONNECTIVITY_SEND_ID 0xB0    // 板件通信发送ID
#define BOARD_CONNECTIVITY_RECEIVE_ID 0xC0 // 板件通信接收ID

typedef enum
{
    BOARD_CONNECTIVITY_CAN_1 = 0,
    BOARD_CONNECTIVITY_CAN_2 = 1,
    BOARD_CONNECTIVITY_USART = 3,
} BoardConnectivityType;

struct BoardMsg
{
    BoardConnectivityType type;
    uint16_t id;
    uint8_t len; // 数据长度，不可以超过8
    uint8_t data[8];
} __attribute__((packed));

#ifdef __cplusplus
extern "C"
{
#endif
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
         * @brief 用于存储接收到的板间通信数据。
         */
        BoardMsg BoardMemory_receive[8];

        /**
         * @brief 初始化函数。
         * 将BoardMemory_send和BoardMemory_receive初始化为nullptr。
         */
        void Init(void);

        /**
         * @brief 添加数据到板间通信的内存中。
         * @param data 板间通信的数据。
         * @param id 板间通信的ID。
         * @param len 板间通信的数据长度。
         * @param type 板间通信的类型。
         * @param direction 板间通信的方向。
         */
        void Add2Memory(BoardMsg &msg);

        /**
         * @brief 发送数据。
         * 将BoardMemory_send中的数据发送出去。
         */
        void BoardConnectivity_Send();

        void GetBoardMemory(uint8_t id, uint8_t *data, uint8_t len);

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

#ifdef __cplusplus
}
#endif

#endif // BOARD_CONNECTIVITY_H
