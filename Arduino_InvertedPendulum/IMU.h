#include <Wire.h>

// 加速度/ジャイロセンサーの制御定数。
constexpr int IMU_ADDR = 0x68;
constexpr int IMU_SMPLRT_DIV = 0x19;
constexpr int IMU_CONFIG = 0x1a;
constexpr int IMU_GYRO_CONFIG = 0x1b;
constexpr int IMU_ACCEL_CONFIG = 0x1c;
constexpr int IMU_ACCEL_X_H = 0x3b;
constexpr int IMU_ACCEL_X_L = 0x3c;
constexpr int IMU_ACCEL_Y_H = 0x3d;
constexpr int IMU_ACCEL_Y_L = 0x3e;
constexpr int IMU_ACCEL_Z_H = 0x3f;
constexpr int IMU_ACCEL_Z_L = 0x40;
constexpr int IMU_GYRO_X_H = 0x43;
constexpr int IMU_GYRO_X_L = 0x44;
constexpr int IMU_GYRO_Y_H = 0x45;
constexpr int IMU_GYRO_Y_L = 0x46;
constexpr int IMU_GYRO_Z_H = 0x47;
constexpr int IMU_GYRO_Z_L = 0x48;
constexpr int IMU_PWR_MGMT_1 = 0x6b;
constexpr int IMU_WHO_AM_I = 0x75;

constexpr int X = 0;
constexpr int Y = 1;
constexpr int Z = 2;

// 加速度/ジャイロセンサーへのコマンド送信。
void writeIMU(byte reg, byte data)
{
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

// 加速度/ジャイロセンサーからのデータ読み込み。
uint8_t readIMU(byte reg)
{
    Wire.beginTransmission(IMU_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(IMU_ADDR, 1 /*length*/, false);
    byte data = Wire.read();
    Wire.endTransmission(true);
    return data;
}

double getAccel(int n)
{
    switch (n)
    {
    case X:
        return (double)((readIMU(IMU_ACCEL_X_H) << 8) | readIMU(IMU_ACCEL_X_L)) / 16384.0;
    case Y:
        return (double)((readIMU(IMU_ACCEL_Y_H) << 8) | readIMU(IMU_ACCEL_Y_L)) / 16384.0;
    case Z:
        return (double)((readIMU(IMU_ACCEL_Z_H) << 8) | readIMU(IMU_ACCEL_Z_L)) / 16384.0;
    }
}

double getGyro(int n)
{
    switch (n)
    {
    case X:
        return (double)((readIMU(IMU_GYRO_X_H) << 8) | readIMU(IMU_GYRO_X_L)) / 131.0;
    case Y:
        return (double)((readIMU(IMU_GYRO_Y_H) << 8) | readIMU(IMU_GYRO_Y_L)) / 131.0;
    case Z:
        return (double)((readIMU(IMU_GYRO_Z_H) << 8) | readIMU(IMU_GYRO_Z_L)) / 131.0;
    }
}
