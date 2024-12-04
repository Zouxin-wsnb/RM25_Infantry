#ifndef TIME_HPP
#define TIME_HPP

#include "stm32f4xx.h"

class Time
{
private:
    uint32_t CurrentTick;
    uint32_t MsPerTick;

    static Time* Instance()
    {
        static Time instance;
        return &instance;
    }

    void SysTick_Init(uint32_t tick_ms)
    {
	    
    }

public:
    Time() : CurrentTick(0),
             MsPerTick(1)
    {
    }

    static void Init(uint32_t msPerTick)
    {
        Instance()->MsPerTick = msPerTick; 
        Instance()->SysTick_Init(msPerTick);
    }
    static uint32_t GetMsPerTick(){ return Instance()->MsPerTick; }
    static uint32_t GetTick(){ return Instance()->CurrentTick; }
    static void Tick(){ ++Instance()->CurrentTick; }
};

#endif