#include <flight_context.hpp>

#include <Arduino.h>

#include <hardware_output.hpp>
#include <flag_manager.hpp>
#include <sensors.hpp>
#include <intercom.hpp>

#include <sensor_data_manager.hpp>
extern DataManager systemData;


// Enter callbacks for each state
void FlightContextManagerCallbacks::ArmedEnterCallback()
{
    // Initialize hardware and sensors
    Serial.begin(115200);
    Serial.println(F("Flight Context: Entering ARMED state. Initializing..."));

    BUZZER::init();
    LED::init();
    PYRO::init();

    // Initialize sensors
    Wire.begin(4, 5);

    if (BMP::init())    BUZZER::Success();
    else    BUZZER::Error();

    if (MPU::init())    BUZZER::Success();
    else    BUZZER::Error();
    
    // Set initial LED color
    LED::setColor(0, 255, 0); // Green
    Serial.println(F("Flight Context: ARMED state initialized."));

    // Initialize intercom
    Intercom::init();
}

void FlightContextManagerCallbacks::EngineEnterCallback()
{
    Serial.println(F("Engine ignition! Let\'s fly!"));
    LED::setColor(255, 165, 0); // Orange
    Intercom::SendPacket(Intercom::messages::MSG_STATE_CHANGETO_ENGINE);
}

void FlightContextManagerCallbacks::FreeAscentEnterCallback()
{
    Serial.println(F("Engine off."));
    LED::setColor(0, 0, 255); // Blue
    Intercom::SendPacket(Intercom::messages::MSG_STATE_CHANGETO_FREE_ASCENT);
}

void FlightContextManagerCallbacks::ApogeeEnterCallback()
{
    Serial.println(F("Apogee reached!"));
    LED::setColor(255, 0, 0); // Red
    Intercom::SendPacket(Intercom::messages::MSG_STATE_CHANGETO_APOGEE);
}

void FlightContextManagerCallbacks::FreeFallEnterCallback()
{
    Serial.println(F("Free fall! This should not last long."));
    LED::setColor(255, 0, 255); // Magenta
    Intercom::SendPacket(Intercom::messages::MSG_STATE_CHANGETO_FREE_FALL);
}

void FlightContextManagerCallbacks::DrogueDescentEnterCallback()
{
    Serial.println(F("Drogue deployed!"));
    LED::setColor(255, 255, 0); // Yellow
    Intercom::SendPacket(Intercom::messages::MSG_STATE_CHANGETO_DROGUE_DESCENT);
}


void FlightContextManagerCallbacks::MainDescentEnterCallback()
{
    Serial.println(F("Main parachute deployed!"));
    LED::setColor(0, 255, 255); // Cyan
    Intercom::SendPacket(Intercom::messages::MSG_STATE_CHANGETO_MAIN_DESCENT);
}

void FlightContextManagerCallbacks::TerminalVelocityEnterCallback()
{
    Serial.println(F("Terminal velocity reached!"));
    LED::setColor(128, 0, 128); // Purple
    Intercom::SendPacket(Intercom::messages::MSG_STATE_CHANGETO_TERMINAL_VELOCITY);
}

void FlightContextManagerCallbacks::TouchdownEnterCallback()
{
    Serial.println(F("Touchdown!"));
    LED::setColor(128, 128, 0); // Olive
    Intercom::SendPacket(Intercom::messages::MSG_STATE_CHANGETO_TOUCHDOWN);
}

void FlightContextManagerCallbacks::RecoveryEnterCallback()
{
    Serial.println(F("Waiting for recovery..."));
    LED::setColor(0, 128, 0); // Dark Green
    Intercom::SendPacket(Intercom::messages::MSG_STATE_CHANGETO_RECOVERY);
}

void FlightContextManagerCallbacks::ErrorEnterCallback()
{
    Serial.println(F("Fuck!"));
    LED::setColor(255, 0, 0); // Red
    Intercom::SendPacket(Intercom::messages::MSG_STATE_CHANGETO_ERROR);
}

// State exit
void FlightContextManagerCallbacks::ArmedExitCallback() {return; /* Do nothing for now */}
void FlightContextManagerCallbacks::EngineExitCallback() {return; /* Do nothing for now */}
void FlightContextManagerCallbacks::FreeAscentExitCallback() {return; /* Do nothing for now */}
void FlightContextManagerCallbacks::ApogeeExitCallback() {return; /* Do nothing for now */}
void FlightContextManagerCallbacks::FreeFallExitCallback() {return; /* Do nothing for now */}
void FlightContextManagerCallbacks::DrogueDescentExitCallback() {return; /* Do nothing for now */}
void FlightContextManagerCallbacks::MainDescentExitCallback() {return; /* Do nothing for now */}
void FlightContextManagerCallbacks::TerminalVelocityExitCallback() {return; /* Do nothing for now */}
void FlightContextManagerCallbacks::TouchdownExitCallback() {return; /* Do nothing for now */}
void FlightContextManagerCallbacks::RecoveryExitCallback() {return; /* Do nothing for now */}
void FlightContextManagerCallbacks::ErrorExitCallback() {return; /* Do nothing for now */}

// State callback
void FlightContextManagerCallbacks::ArmedCallback(void)
{
    // Actually, there is really nothing to do here.
    // Every 500ms or so, we will beat to the partner, so they can see we are alive.
    Intercom::SendPacket(Intercom::messages::MSG_BEAT);
    // Since the receive callback will handle whether the partner is alive or not,
    // There is absolutely no need to do anything here.
    // Yes. As you noticed, I am writing this cause there is nothing really to do here,
    // But leaving only this empty will look odd.
}

void FlightContextManagerCallbacks::EngineCallback(void)
{
    // Let's go!
    // We are in the air, and we are accelerating.
    Intercom::SendPacket(Intercom::messages::MSG_BEAT);

    // First, we should read the sensors.
    BMP::read(systemData.bmpTemperature, systemData.bmpPressure);

    bool mpu_success = MPU::read(
        systemData.quat,
        systemData.world_accel_xyz,
        systemData.body_accel_xyz,
        systemData.gxyz,
        systemData.grav
    );

    // Compute the values

    // 1. Derived from BMP280
    float hpaPressure = systemData.bmpPressure;
    systemData.bmpAltitudeInstant = 44330.0f * (1.0f - pow(hpaPressure / systemData.seaLevelPressure, 0.1903f));
    systemData.bmpAltitudeCircularQueue[systemData.bmpAltitudeQueueIndex] = systemData.bmpAltitudeInstant;
    systemData.bmpAltitudeQueueIndex = (systemData.bmpAltitudeQueueIndex + 1) % (sizeof(systemData.bmpAltitudeCircularQueue) / sizeof(systemData.bmpAltitudeCircularQueue[0]));
    double sum = 0.0; int count = 0;
    for (size_t i = 0; i < sizeof(systemData.bmpAltitudeCircularQueue) / sizeof(systemData.bmpAltitudeCircularQueue[0]); ++i) {
        sum += systemData.bmpAltitudeCircularQueue[i];
        if (systemData.bmpAltitudeCircularQueue[i] != 0.0f) { // Using == is justified here, since we want the 'exact' 0 value. We initialized the array with 0.0f.
            count++;
        }
    }
    systemData.bmpAltitude = (sum / static_cast<float>(count));
    systemData.bmpTemperatureCelsius = systemData.bmpTemperature;

    // 2. Derived from MPU6050
    
}