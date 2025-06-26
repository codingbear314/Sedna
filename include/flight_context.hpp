#pragma once

#ifndef FLIGHT_CONTEXT_HPP
#define FLIGHT_CONTEXT_HPP

#include <Arduino.h>
#include "flag_manager.hpp" // enum SystemState
extern StatusFlags statusFlags;

#include "sensors.hpp"

struct FlightContextManager
{
    SystemState lastState = SystemState::STATE_ERROR; // Default to ERROR state
    // current is statusFlags.flags.status;
    SystemState current = SystemState::STATE_ERROR; // Default to ERROR state
    // Remember to update the flags also

    void changeState(SystemState new_state)
    {
        lastState = current;
        current = new_state;
        statusFlags.flags.status = new_state;
        onExitState(lastState);
        onEnterState(current);
    }

    void onEnterState(SystemState state);
    void onExitState(SystemState state);
    void callback(SystemState state);
};

namespace FlightContextManagerCallbacks
{
    void ArmedEnterCallback();
    void EngineEnterCallback();
    void FreeAscentEnterCallback();
    void ApogeeEnterCallback();
    void FreeFallEnterCallback();
    void DrogueDescentEnterCallback();
    void MainDescentEnterCallback();
    void TerminalVelocityEnterCallback();
    void TouchdownEnterCallback();
    void RecoveryEnterCallback();
    void ErrorEnterCallback();

    void ArmedExitCallback();
    void EngineExitCallback();
    void FreeAscentExitCallback();
    void ApogeeExitCallback();
    void FreeFallExitCallback();
    void DrogueDescentExitCallback();
    void MainDescentExitCallback();
    void TerminalVelocityExitCallback();
    void TouchdownExitCallback();
    void RecoveryExitCallback();
    void ErrorExitCallback();

    void ArmedCallback();
    void EngineCallback();
    void FreeAscentCallback();
    void ApogeeCallback();
    void FreeFallCallback();
    void DrogueDescentCallback();
    void MainDescentCallback();
    void TerminalVelocityCallback();
    void TouchdownCallback();
    void RecoveryCallback();
    void ErrorCallback();
}

void FlightContextManager::onEnterState(SystemState state)
{
    switch (state)
    {
        case SystemState::STATE_ARMED:
            FlightContextManagerCallbacks::ArmedEnterCallback();
            break;
        case SystemState::STATE_ENGINE:
            FlightContextManagerCallbacks::EngineEnterCallback();
            break;
        case SystemState::STATE_FREE_ASCENT:
            FlightContextManagerCallbacks::FreeAscentEnterCallback();
            break;
        case SystemState::STATE_APOGEE:
            FlightContextManagerCallbacks::ApogeeEnterCallback();
            break;
        case SystemState::STATE_FREE_FALL:
            FlightContextManagerCallbacks::FreeFallEnterCallback();
            break;
        case SystemState::STATE_DROGUE_DESCENT:
            FlightContextManagerCallbacks::DrogueDescentEnterCallback();
            break;
        case SystemState::STATE_MAIN_DESCENT:
            FlightContextManagerCallbacks::MainDescentEnterCallback();
            break;
        case SystemState::STATE_TERMINAL_VELOCITY:
            FlightContextManagerCallbacks::TerminalVelocityEnterCallback();
            break;
        case SystemState::STATE_TOUCHDOWN:
            FlightContextManagerCallbacks::TouchdownEnterCallback();
            break;
        case SystemState::STATE_RECOVERY:
            FlightContextManagerCallbacks::RecoveryEnterCallback();
            break;
        case SystemState::STATE_ERROR:
            FlightContextManagerCallbacks::ErrorEnterCallback();
            break;
    }
}

void FlightContextManager::onExitState(SystemState state)
{
    switch (state)
    {
        case SystemState::STATE_ARMED:
            FlightContextManagerCallbacks::ArmedExitCallback();
            break;
        case SystemState::STATE_ENGINE:
            FlightContextManagerCallbacks::EngineExitCallback();
            break;
        case SystemState::STATE_FREE_ASCENT:
            FlightContextManagerCallbacks::FreeAscentExitCallback();
            break;
        case SystemState::STATE_APOGEE:
            FlightContextManagerCallbacks::ApogeeExitCallback();
            break;
        case SystemState::STATE_FREE_FALL:
            FlightContextManagerCallbacks::FreeFallExitCallback();
            break;
        case SystemState::STATE_DROGUE_DESCENT:
            FlightContextManagerCallbacks::DrogueDescentExitCallback();
            break;
        case SystemState::STATE_MAIN_DESCENT:
            FlightContextManagerCallbacks::MainDescentExitCallback();
            break;
        case SystemState::STATE_TERMINAL_VELOCITY:
            FlightContextManagerCallbacks::TerminalVelocityExitCallback();
            break;
        case SystemState::STATE_TOUCHDOWN:
            FlightContextManagerCallbacks::TouchdownExitCallback();
            break;
        case SystemState::STATE_RECOVERY:
            FlightContextManagerCallbacks::RecoveryExitCallback();
            break;
        case SystemState::STATE_ERROR:
            FlightContextManagerCallbacks::ErrorExitCallback();
            break;
    }
}

void FlightContextManager::callback(SystemState state)
{
    switch (state)
    {
        case SystemState::STATE_ARMED:
            FlightContextManagerCallbacks::ArmedCallback();
            break;
        case SystemState::STATE_ENGINE:
            FlightContextManagerCallbacks::EngineCallback();
            break;
        case SystemState::STATE_FREE_ASCENT:
            FlightContextManagerCallbacks::FreeAscentCallback();
            break;
        case SystemState::STATE_APOGEE:
            FlightContextManagerCallbacks::ApogeeCallback();
            break;
        case SystemState::STATE_FREE_FALL:
            FlightContextManagerCallbacks::FreeFallCallback();
            break;
        case SystemState::STATE_DROGUE_DESCENT:
            FlightContextManagerCallbacks::DrogueDescentCallback();
            break;
        case SystemState::STATE_MAIN_DESCENT:
            FlightContextManagerCallbacks::MainDescentCallback();
            break;
        case SystemState::STATE_TERMINAL_VELOCITY:
            FlightContextManagerCallbacks::TerminalVelocityCallback();
            break;
        case SystemState::STATE_TOUCHDOWN:
            FlightContextManagerCallbacks::TouchdownCallback();
            break;
        case SystemState::STATE_RECOVERY:
            FlightContextManagerCallbacks::RecoveryCallback();
            break;
        case SystemState::STATE_ERROR:
            FlightContextManagerCallbacks::ErrorCallback();
            break;
    }
}

// Implement these on flight_context.cpp

#endif // FLIGHT_CONTEXT_HPP 