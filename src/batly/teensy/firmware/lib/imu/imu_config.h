#ifndef _IMU_CONFIG_H_
#define _IMU_CONFIG_H_

#include "I2Cdev.h"

#define G_TO_ACCEL 9.81
#define MGAUSS_TO_UTESLA 0.1
#define UTESLA_TO_TESLA 0.000001

#ifdef USE_MPU6050_IMU
    #include "MPU6050.h"
    #include "fake_mag.h"

    #define ACCEL_SCALE 1 / 16384 // LSB/g
    #define GYRO_SCALE 1 / 131 // LSB/(deg/s)
    #define MAG_SCALE 0.3 // uT/LSB
    
    MPU6050 accelerometer;
    MPU6050 gyroscope;    
    FakeMag magnetometer;
#endif

#endif

