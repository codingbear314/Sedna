#pragma once

#ifndef __ACTIONS_HPP__
#define __ACTIONS_HPP__

#include <Arduino.h>
#include <Wire.h>
#include "flag_manager.hpp"

extern StatusFlags statusFlags;

namespace BUZZER
{
    constexpr uint8_t BUZZER_PIN = 47;
    void init()
    {
        pinMode(BUZZER_PIN, OUTPUT);
    }
    void play(uint16_t frequency, uint32_t duration)
    {
        int delayPeriod = 1000000 / frequency / 2;
        long cycles = static_cast<long>(frequency) * duration / 1000;

        for (long i = 0; i < cycles; i++)
        {
            digitalWrite(BUZZER_PIN, HIGH);
            delayMicroseconds(delayPeriod);
            digitalWrite(BUZZER_PIN, LOW);
            delayMicroseconds(delayPeriod);
        }
    }
}

#include <Adafruit_NeoPixel.h>
namespace LED
{
    constexpr uint8_t LED_PIN = 38;
    Adafruit_NeoPixel LED_obj(1, LED_PIN, NEO_GRB + NEO_KHZ800);

    void init()
    {
        LED_obj.begin();
        LED_obj.setBrightness(50);
        LED_obj.show();
    }
    
    void setColor(uint8_t r, uint8_t g, uint8_t b)
    {
        LED_obj.setPixelColor(0, LED_obj.Color(r, g, b));
        LED_obj.show();
    }
}

namespace PYRO
{
    constexpr uint8_t drouge = 6;
    constexpr uint8_t main = 7;

    void init()
    {
        pinMode(drouge, OUTPUT);
        pinMode(main, OUTPUT);
        digitalWrite(drouge, LOW);
        digitalWrite(main, LOW);
    }
    
    void deployDrouge()
    {
        digitalWrite(drouge, HIGH);
        statusFlags.flags.drouge_deployed = 1;
        Serial.println(F("Drouge deployed!"));
    }

    void retractDrouge()
    {
        digitalWrite(drouge, LOW);
        statusFlags.flags.drouge_deployed = 0;
        Serial.println(F("Drouge retracted!"));
    }

    void deployMain()
    {
        digitalWrite(main, HIGH);
        statusFlags.flags.main_deployed = 1;
        Serial.println(F("Main parachute deployed!"));
    }

    void retractMain()
    {
        digitalWrite(main, LOW);
        statusFlags.flags.main_deployed = 0;
        Serial.println(F("Main parachute retracted!"));
    }
}

#endif // __ACTIONS_HPP__