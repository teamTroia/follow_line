#include "../include/imuUtils.h"
#include <SoftWire.h>
#include "../include/types.h"

#define I2C_SDA PB9
#define I2C_SCL PB8

SoftWire WIRE1(PB8, PB9, I2C_FAST_MODE);
#define Wire WIRE1

IMU::IMU() {}
void IMU::init_mpu() {
    Wire.begin();
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x6B);
    Wire.write(0);
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_ADDR);  // SET RANGE GYRO
    Wire.write(0x1B);                  // endereço do registrador
    Wire.write(0b00001000);            // valor para 500 g/s
    Wire.endTransmission(true);

    Wire.beginTransmission(MPU_ADDR);  // SET BAND WIDTH
    Wire.write(0x1A);                  // endereço do registrador
    Wire.write(0b00000100);            // valor para 21Hz
    Wire.endTransmission(true);
}

float IMU::readAngularSpeed() {
    int16_t AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ;
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 14, true);
    AcX = Wire.read() << 8 |
          Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)
    AcY = Wire.read() << 8 |
          Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
    AcZ = Wire.read() << 8 |
          Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
    Tmp = Wire.read() << 8 |
          Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
    GyX = Wire.read() << 8 |
          Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
    GyY = Wire.read() << 8 |
          Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
    GyZ = Wire.read() << 8 |
          Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)

    float gyro_scale = 1;
    if (false) gyro_scale = 131;
    if (true) gyro_scale = 65.5;
    if (false) gyro_scale = 32.8;
    if (false) gyro_scale = 16.4;

    float gyroX = ((float)GyX) / gyro_scale;
    float gyroY = ((float)GyY) / gyro_scale;
    float gyroZ = ((float)GyZ) / gyro_scale;

    gyroX *= (0.017453293F);
    gyroY *= (0.017453293F);
    gyroZ *= (0.017453293F);
    Serial.print("gyroZ: ");
    Serial.println(gyroZ);
    return gyroZ;
}