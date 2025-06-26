#pragma once

#ifndef __SENSOR_DATA_MANAGER_HPP__
#define __SENSOR_DATA_MANAGER_HPP__

#include <Arduino.h>
#include "sensors.hpp"
#include "flag_manager.hpp"
extern StatusFlags statusFlags;

struct DataManager
{
    // Raw sensor data
    // 1. BMP280: temperature, pressure
    float bmpTemperature = 0.0f; // [Celsius]
    float bmpPressure = 0.0f; // [hPa]
    // 2. MPU6050: quaternion, world accel, body accel, gyro,
    //    gravity vector, body-frame accel
    float quat[4] = {0.0f, 0.0f, 0.0f, 0.0f}; // w, x, y, z [quaternion]
    float world_accel_xyz[3] = {0.0f, 0.0f, 0.0f}; // x, y, z [m/s^2]
    float last_world_accel_xyz[3] = {0.0f, 0.0f, 0.0f}; // Last world accel x, y, z [m/s^2]
    float body_accel_xyz[3] = {0.0f, 0.0f, 0.0f}; // x, y, z [m/s^2]
    float gxyz[3] = {0.0f, 0.0f, 0.0f}; // gyro x, y, z [deg/s]
    float grav[3] = {0.0f, 0.0f, 0.0f}; // gravity x, y, z [m/s^2]

    // Variables needed to process data
    float bmpAltitudeInstant = 0.0f; // Altitude calculated from BMP280 pressure [meters]
    float bmpAltitudeCircularQueue[16] = {0.0f}; // Circular queue for altitude. This is for moving average. [meters]
    uint8_t bmpAltitudeQueueIndex = 0; // Index for circular queue

    float estimatedDrogueParachute_dragCoefficient = 0.0f; // Estimated drag coefficient for drogue parachute [dimensionless]
    const float MainParachute_dragCoefficient = 0; // This is known. Fill this in the main parachute enter callback. [dimensionless]
    const float seaLevelPressure = 1013.25f; // Sea level pressure in hPa [hPa]

    float estimateX_velocity = 0.0f; // Estimated X velocity, known by integration of world_accel_xyz[0] [m/s]
    float estimateX_last_velocity = 0.0f; // Last X velocity, known by integration of world_accel_xyz[0] [m/s]
    float estimateY_velocity = 0.0f; // Estimated Y velocity, known by integration of world_accel_xyz[1] [m/s]
    float estimateY_last_velocity = 0.0f; // Last Y velocity, known by integration of world_accel_xyz[1] [m/s]
    float estimateZ_velocity = 0.0f; // Estimated Z velocity, known by integration of world_accel_xyz[2] [m/s]
    float estimateZ_last_velocity = 0.0f; // Last Z velocity, known by integration of world_accel_xyz[2] [m/s]
    float estimateX_position = 0.0f; // Estimated X position, known by integration of estimateX_velocity [m]
    float estimateY_position = 0.0f; // Estimated Y position, known by integration of estimateY_velocity [m]
    float estimateZ_position = 0.0f; // Estimated Z position, known by integration of estimateZ_velocity [m]

    // Processed sensor data
    float bmpAltitude = 0.0f; // [meters]
    float bmpTemperatureCelsius = 0.0f; // [Celsius]
    
    float fusedAltitude = 0.0f; // [meters]
    // Fused altitude from BMP280 and MPU6050. This is NOT trustable, but it is used for integration.

    uint32_t lastUpdateTime = 0; // Last update time in milliseconds
    uint32_t lastUpdateTimeBMP = 0; // Last update time for BMP280
    uint32_t lastUpdateTimeMPU = 0; // Last update time for MPU6050
};

extern DataManager systemData;

#endif // __SENSOR_DATA_MANAGER_HPP__