// 加速度/ジャイロセンサーの制御定数。
#define MPU6050_ADDR 0x68       // MPU-6050 device address
#define MPU6050_SMPLRT_DIV 0x19 // MPU-6050 register address
#define MPU6050_CONFIG 0x1a
#define MPU6050_GYRO_CONFIG 0x1b
#define MPU6050_ACCEL_CONFIG 0x1c
#define MPU6050_ACCEL_XOUT_H 0x3b
#define MPU6050_ACCEL_XOUT_L 0x3c
#define MPU6050_ACCEL_YOUT_H 0x3d
#define MPU6050_ACCEL_YOUT_L 0x3e
#define MPU6050_ACCEL_ZOUT_H 0x3f
#define MPU6050_ACCEL_ZOUT_L 0x40
#define MPU6050_GYRO_XOUT_H 0x43
#define MPU6050_GYRO_XOUT_L 0x44
#define MPU6050_GYRO_YOUT_H 0x45
#define MPU6050_GYRO_YOUT_L 0x46
#define MPU6050_GYRO_ZOUT_H 0x47
#define MPU6050_GYRO_ZOUT_L 0x48
#define MPU6050_PWR_MGMT_1 0x6b
#define MPU6050_WHO_AM_I 0x75

// 加速度/ジャイロセンサーへのコマンド送信。
void writeMPU6050(byte reg, byte data)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.write(data);
    Wire.endTransmission();
}

// 加速度/ジャイロセンサーからのデータ読み込み。
byte readMPU6050(byte reg)
{
    Wire.beginTransmission(MPU6050_ADDR);
    Wire.write(reg);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU6050_ADDR, 1 /*length*/, false);
    byte data = Wire.read();
    Wire.endTransmission(true);
    return data;
}