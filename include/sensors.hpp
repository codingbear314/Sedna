#pragma once

#ifndef __SENSORS_HPP__
#define __SENSORS_HPP__

#include <Arduino.h>
#include <Wire.h>

#include "flag_manager.hpp"
extern StatusFlags statusFlags;

// BMP280 sensor
#include <Adafruit_BMP280.h>
namespace BMP
{
    Adafruit_BMP280 bmp;

    bool init()
    {
        if (!bmp.begin(0x76))
        {
            statusFlags.flags.bmp_inited = 0;
            Serial.println(F("Could not find a valid BMP280 sensor, check wiring!"));
            return false;
        }
        else
        {
            statusFlags.flags.bmp_inited = 1;
            Serial.println(F("BMP280 sensor good to go!"));
            return true;
        }
    }

    void read(float& temperature, float& pressure)
    {
        temperature = bmp.readTemperature(); // Celsius
        pressure = bmp.readPressure() / 100.0F; // hPa
    }

    float readAltitude(float seaLevelPressure = 1013.25F)
    {
        return bmp.readAltitude(seaLevelPressure); // meters
    }
}

// MPU6050 sensor
#include "I2Cdev.h"
#include "MPU6050_6Axis_MotionApps20.h"
namespace MPU
{
    MPU6050 mpu;
    constexpr auto INTERRUPT_PIN = 8;

    volatile bool mpuInterrupt = false;
    void dmpDataReady() { mpuInterrupt = true; }

    uint8_t Queue_Buffer[64];

    bool init()
    {
        Serial.println(F("Initializing MPU 6050..."));
        mpu.initialize();
        pinMode(INTERRUPT_PIN, INPUT);

        Serial.println(F("Testing device connections..."));
        if (mpu.testConnection()) {
            Serial.println(F("MPU6050 connection successful"));
            statusFlags.flags.mpu_inited = 1;
        } else {
            Serial.println(F("MPU6050 connection failed"));
            statusFlags.flags.mpu_inited = 0;
            return false;
        }

        Serial.println(F("Initializing DMP..."));
        uint8_t devStatus = mpu.dmpInitialize();

        if (devStatus == 0) {
            mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_16);
            mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
            mpu.CalibrateAccel(12);
            mpu.CalibrateGyro(12);
            mpu.setDMPEnabled(true);
            statusFlags.flags.mpu_inited = 1;
            Serial.println(F("DMP ready!"));
        } else {
            Serial.print(F("DMP Initialization failed (code "));
            Serial.print(devStatus);
            Serial.println(F(")"));
            statusFlags.flags.mpu_inited = 0;
            return false;
        }
        
        // Set up interrupt
        attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
        return true;
    }

    void dmpGetLinearAccelInWorld(VectorFloat* v, VectorFloat* vReal, Quaternion* q) {
        // Rotate body-frame linear acceleration into world frame
        float qw = q->w, qx = q->x, qy = q->y, qz = q->z;

        float x = vReal->x, y = vReal->y, z = vReal->z;

        // Rotate vector by quaternion: v' = q * v * q^-1
        v->x = x * (1 - 2 * qy*qy - 2 * qz*qz) + y * (2 * qx*qy - 2 * qz*qw) + z * (2 * qx*qz + 2 * qy*qw);
        v->y = x * (2 * qx*qy + 2 * qz*qw) + y * (1 - 2 * qx*qx - 2 * qz*qz) + z * (2 * qy*qz - 2 * qx*qw);
        v->z = x * (2 * qx*qz - 2 * qy*qw) + y * (2 * qy*qz + 2 * qx*qw) + z * (1 - 2 * qx*qx - 2 * qy*qy);
    }

    /**
     * Reads fully aligned DMP output + computes world-frame linear accel.
     * 
     * @param quat Pointer to an array of 4 floats for quaternion (w, x, y, z).
     * @param world_accel_xyz Pointer to an array of 3 floats for world-frame linear accel (m/s^2).
     * @param gxyz Pointer to an array of 3 floats for gyro angular velocity (deg/s).
     * @param grav Pointer to an array of 3 floats for gravity vector (m/s^2).
     * @return true if data was read successfully, false otherwise.
     */
    bool read(float* quat, float* world_accel_xyz, float* body_accel_xyz, float* gxyz, float* grav)
    {
        if (!statusFlags.flags.mpu_inited) return false;

        // FIFO overflow?
        if (mpu.getIntStatus() & 0x10) {
            mpu.resetFIFO();
            Serial.println(F("FIFO overflow"));
            return false;
        }

        // New DMP packet available?
        if (mpuInterrupt && mpu.dmpGetCurrentFIFOPacket(Queue_Buffer)) {
            Quaternion    q;
            VectorFloat   gravity;
            VectorInt16   accelRaw;   // <-- for dmpGetAccel
            VectorInt16   gyroRaw;    // <-- for dmpGetGyro

            // Quaternion & gravity from DMP
            mpu.dmpGetQuaternion(&q, Queue_Buffer);
            mpu.dmpGetGravity   (&gravity, &q);

            // Body-frame accel & gyro from same packet
            mpu.dmpGetAccel(&accelRaw, Queue_Buffer);
            mpu.dmpGetGyro (&gyroRaw,  Queue_Buffer);

            // Fill quaternion
            quat[0] = q.w;
            quat[1] = q.x;
            quat[2] = q.y;
            quat[3] = q.z;

            // Gravity vector in m/s^2
            grav[0] = gravity.x * 9.81f;
            grav[1] = gravity.y * 9.81f;
            grav[2] = gravity.z * 9.81f;

            // Body-frame accel → m/s^2 (±16g = 2048 LSB/g)
            float bodyAcc[3] = {
                accelRaw.x / 2048.0f * 9.81f,
                accelRaw.y / 2048.0f * 9.81f,
                accelRaw.z / 2048.0f * 9.81f
            };

            body_accel_xyz[0] = bodyAcc[0];
            body_accel_xyz[1] = bodyAcc[1];
            body_accel_xyz[2] = bodyAcc[2];

            // Rotate into world frame: v_world = q * v_body * q*
            float vx =  q.w * bodyAcc[0] + q.y * bodyAcc[2] - q.z * bodyAcc[1];
            float vy =  q.w * bodyAcc[1] + q.z * bodyAcc[0] - q.x * bodyAcc[2];
            float vz =  q.w * bodyAcc[2] + q.x * bodyAcc[1] - q.y * bodyAcc[0];
            float vw = -q.x * bodyAcc[0] - q.y * bodyAcc[1] - q.z * bodyAcc[2];

            float worldAcc[3] = {
                vx * q.w - vw * q.x + vy * q.z - vz * q.y,
                vy * q.w - vw * q.y + vz * q.x - vx * q.z,
                vz * q.w - vw * q.z + vx * q.y - vy * q.x
            };

            // Linear acceleration = worldAcc - gravity
            world_accel_xyz[0] = worldAcc[0] - grav[0];
            world_accel_xyz[1] = worldAcc[1] - grav[1];
            world_accel_xyz[2] = worldAcc[2] - grav[2];

            // Angular velocity in °/s  (±2000°/s = 16.4 LSB/°/s)
            gxyz[0] = gyroRaw.x / 16.4f;
            gxyz[1] = gyroRaw.y / 16.4f;
            gxyz[2] = gyroRaw.z / 16.4f;

            mpuInterrupt = false;
            return true;
        }

        return false;
    }
}

#endif // __SENSORS_HPP__