#pragma once

#include "CPPI2C/cppi2c.h"
#include "lsm6ds3tr-c_reg.h"

class LSM6DS3
{
private:
    CPPI2C::I2c i2c{I2C_NUM_0};
    uint8_t slave_address;
    stmdev_ctx_t dev_ctx;

public:
    LSM6DS3(i2c_port_t i2c_port = I2C_NUM_0, uint8_t slaveAddress = LSM6DS3TR_C_I2C_ADD_L);

    int begin();
    void setDataReadyInt2();

    void end();

    // Accelerometer
    int readAcceleration(float& x, float& y, float& z); // Results are in g (earth gravity).
    float accelerationSampleRate(); // Sampling rate of the sensor.
    int accelerationAvailable(); // Check for available data from accelerometer

    // Gyroscope
    int readGyroscope(float& x, float& y, float& z); // Results are in degrees/second.
    float gyroscopeSampleRate(); // Sampling rate of the sensor.
    int gyroscopeAvailable(); // Check for available data from gyroscope
};
