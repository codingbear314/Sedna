#ifndef __FLAG_MANAGER_HPP__
#define __FLAG_MANAGER_HPP__

#include <Arduino.h>

/**
 * 1. ARMED - On ground. IDLE is not a thing, cause why do we need it. We have enough battery life.
 * 2. ENGINE - Engine started, the rocket is actively accelerating. The mpu reads large values.
 * 3. FREE_ASCENT - Engine off. The rocket is getting slower, and mpu reads gravity + air resistance.
 * 4. APOGEE - Rocket reached full altitude. The orientation flips, and z velocity becomes almost zero. The z velocity flips.
 * 5. FREE_FALL - The rocket is free falling. This should not go long, since drogue should deploy at apogee.
 * 6. DROGUE_DESCENT - The drogue is out. The rocket is falling according to the ODE. The MPU reads values towards top of rocket (negative, in world coords), smaller than g but still is not zero.
 * 7. MAIN_DESCENT - The main parachute is out. The rocket is falling according to the ODE. The MPU reads values towards top of rocket, which is smaller than phase 6. 
 * 8. TERMINAL_VELOCITY - The rocket reached terminal velocity. Now, it's very easy to predict it's path. At this point, we can know where the rocket will fall, and when it will touchdown.
 * 9. TOUCHDOWN - The rocket is down. After it stablize, we switch to phase 10.
 * 10. RECOVERY - We now wait for recovery. We can use many methods for recovery. We broadcast expected location by both sedna boards. And we wait. When the ground station ping, we send them something, so we can use a directional antenna if we are desperate.
 */

enum SystemState : uint8_t {
    STATE_ARMED = 0,
    STATE_ENGINE,
    STATE_FREE_ASCENT,
    STATE_APOGEE,
    STATE_FREE_FALL,
    STATE_DROGUE_DESCENT,
    STATE_MAIN_DESCENT,
    STATE_TERMINAL_VELOCITY,
    STATE_TOUCHDOWN,
    STATE_RECOVERY,
    STATE_ERROR,
};

union StatusFlags {
    struct {
        uint16_t bmp_inited   : 1;
        uint16_t mpu_inited   : 1;
        uint16_t sd_inited    : 1;
        uint16_t flash_inited : 1;
        uint16_t status       : 4; // SystemState enum
        uint16_t drouge_deployed : 1;
        uint16_t main_deployed   : 1;
        uint16_t esp_now_inited : 1;
        uint16_t peer_dead     : 1;
        uint16_t reserved     : 4;
    } flags;
    uint16_t state;
} __attribute__((packed));

inline const char* getStateName(uint8_t s) {
    switch (s) {
        case STATE_ARMED: return "ARMED";
        case STATE_ENGINE: return "ENGINE";
        case STATE_FREE_ASCENT: return "FREE_ASCENT";
        case STATE_APOGEE: return "APOGEE";
        case STATE_FREE_FALL: return "FREE_FALL";
        case STATE_DROGUE_DESCENT: return "DROGUE_DESCENT";
        case STATE_MAIN_DESCENT: return "MAIN_DESCENT";
        case STATE_TERMINAL_VELOCITY: return "TERMINAL_VELOCITY";
        case STATE_TOUCHDOWN: return "TOUCHDOWN";
        case STATE_RECOVERY: return "RECOVERY";
    }
}

#endif // __FLAG_MANAGER_HPP__
