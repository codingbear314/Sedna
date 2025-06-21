#ifndef __FLAG_MANAGER_HPP__
#define __FLAG_MANAGER_HPP__

#include <Arduino.h>

enum SystemState : uint8_t {
    STATE_GROUND = 0,
    STATE_ASCENT,
    STATE_DESCENT,
    STATE_PARACHUTE_1_DEPLOYED,
    STATE_PARACHUTE_2_DEPLOYED,
    STATE_LANDED,
    STATE_ERROR
};

union StatusFlags {
    struct {
        uint16_t bmp_inited   : 1;
        uint16_t mpu_inited   : 1;
        uint16_t sd_inited    : 1;
        uint16_t flash_inited : 1;
        uint16_t status       : 3; // SystemState enum
        uint16_t drouge_deployed : 1;
        uint16_t main_deployed   : 1;
        uint16_t esp_now_inited : 1;
        uint16_t peer_dead     : 1;
        uint16_t reserved     : 5;
    } flags;
    uint16_t state;
} __attribute__((packed));

inline const char* getStateName(uint8_t s) {
    switch (s) {
        case STATE_GROUND: return "GROUND";
        case STATE_ASCENT: return "ASCENT";
        case STATE_DESCENT: return "DESCENT";
        case STATE_PARACHUTE_1_DEPLOYED: return "PARACHUTE_1_DEPLOYED";
        case STATE_PARACHUTE_2_DEPLOYED: return "PARACHUTE_2_DEPLOYED";
        case STATE_LANDED: return "LANDED";
        case STATE_ERROR: return "ERROR";
        default: return "UNKNOWN";
    }
}

#endif // __FLAG_MANAGER_HPP__
