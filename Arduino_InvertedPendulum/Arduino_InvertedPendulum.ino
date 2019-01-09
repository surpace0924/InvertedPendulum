/*
#include <Wire.h>
#include <SoftwareSerial.h>
    SoftwareSerial mySerial(A2, A3); // RX, TX
#include "KalmanFilter.h"
#include "PID.h"
#include "IMU.h"
#include "Driver.h"

#define K1 0.15
#define K2 0 //0.1
#define K3 -0.02
#define K4 0 //0.002

PID pid_x(0, 1, 0);
PID pid(K1, K2, 0);

KalmanFilter kf;
float calib_angle;
float calib_gyro;
float body_v = 0;
float body_x = 0;
long prev_time;

float i = 0;

void setup()
{
    // TCCR3B = (TCCR3B & 0b11111000) | 0x01; //31.37255 [kHz]
    // TCCR4B = (TCCR4B & 0b11111000) | 0x01; //31.37255 [kHz]

    Serial.begin(115200);
    mySerial.begin(57600);

    // モータードライバの初期化。
    pinMode(2, OUTPUT);
    pinMode(3, OUTPUT);
    pinMode(6, OUTPUT);
    pinMode(7, OUTPUT);

    // 加速度/ジャイロセンサーの初期化。
    Wire.begin();
    if (readMPU6050(MPU6050_WHO_AM_I) != 0x68)
    {
        Serial.println("\nWHO_AM_I error.");
        while (true)
            ;
    }

    writeMPU6050(MPU6050_SMPLRT_DIV, 0x07);   // sample rate: 8kHz/(7+1) = 1kHz
    writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
    writeMPU6050(MPU6050_GYRO_CONFIG, 0x00);  // gyro range: ±250dps
    writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
    writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro
    delay(2000);

    // 重力加速度から求めた角度をカルマンフィルタの初期値とする。
    float ax = (readMPU6050(MPU6050_ACCEL_XOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_XOUT_L);
    float ay = (readMPU6050(MPU6050_ACCEL_YOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_YOUT_L);
    float az = (readMPU6050(MPU6050_ACCEL_ZOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_ZOUT_L);
    float gy = 0;
    for (int i = 0; i < 100; i++)
    {
        gy += (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);
    }
    calib_gyro = gy / 131.0 / 100;

    float pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    kf.setAngle(pitch);
    calib_angle = pitch;
    prev_time = micros();
}

void loop()
{

    // 重力加速度から角度を求める。
    float ax = (readMPU6050(MPU6050_ACCEL_XOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_XOUT_L);

    float ay = (readMPU6050(MPU6050_ACCEL_YOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_YOUT_L);

    float az = (readMPU6050(MPU6050_ACCEL_ZOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_ZOUT_L);

    float gy = (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);

    float pitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    float gyro = gy / 131.0 - calib_gyro;
    if (abs(gyro) < 0.5)
        gyro = 0;

    // カルマンフィルタで角度(x,y)を計算する。
    long cur_time = micros();
    float dt = (float)(cur_time - prev_time) / 1000000; // μsec -> sec
    prev_time = cur_time;
    float degY = kf.calcAngle(pitch, gyro, dt);
    degY -= calib_angle;

    float diff_term = K3 * gyro;
    float other_term = pid.calculate(0, degY, dt);

    float motorVal;

    if (diff_term * degY > 0)
    {
        motorVal = other_term;
    }
    else
    {
        motorVal = diff_term + other_term;
    }
    Serial.print(degY);
    Serial.print("\t");

    if (abs(degY) < 17)
        drive(0, motorVal, false);
    else
        drive(0, 0, true);

    delay(10);
}
*/
//////////////////////////////

#include <Wire.h>
#include <SoftwareSerial.h>
#include "KalmanFilter.h"
#include "IMU.h"

SoftwareSerial mySerial(A2, A3); // RX, TX

// 加速度/ジャイロセンサーの制御変数。
KalmanFilter gKfx, gKfy; // カルマンフィルタ。
float gCalibrateY;       // 初期化時の角度。（＝静止角とみなす）
long gPrevMicros;        // loop()間隔の計測用。

// 倒立振子の制御変数。
float gPowerP, gPowerI, gPowerD; // 現在出力値とPID成分。

void setup()
{
    Serial.begin(115200);
    mySerial.begin(57600);

    // 加速度/ジャイロセンサーの初期化。
    Wire.begin();
    if (readMPU6050(MPU6050_WHO_AM_I) != 0x68)
    {
        Serial.println("\nWHO_AM_I error.");
        while (true)
            ;
    }
    writeMPU6050(MPU6050_SMPLRT_DIV, 0x07);   // sample rate: 8kHz/(7+1) = 1kHz
    writeMPU6050(MPU6050_CONFIG, 0x00);       // disable DLPF, gyro output rate = 8kHz
    writeMPU6050(MPU6050_GYRO_CONFIG, 0x00);  // gyro range: ±250dps
    writeMPU6050(MPU6050_ACCEL_CONFIG, 0x00); // accel range: ±2g
    writeMPU6050(MPU6050_PWR_MGMT_1, 0x01);   // disable sleep mode, PLL with X gyro
    delay(500);

    // 重力加速度から求めた角度をカルマンフィルタの初期値とする。
    float ax = (readMPU6050(MPU6050_ACCEL_XOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_XOUT_L);
    float ay = (readMPU6050(MPU6050_ACCEL_YOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_YOUT_L);
    float az = (readMPU6050(MPU6050_ACCEL_ZOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_ZOUT_L);
    float degRoll = atan2(ay, az) * RAD_TO_DEG;
    float degPitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    gKfx.setAngle(degRoll);
    gKfy.setAngle(degPitch);
    gCalibrateY = degPitch;
    gCalibrateY = 0;
    gPrevMicros = micros();
}

void loop()
{

    // 重力加速度から角度を求める。
    float ax = (readMPU6050(MPU6050_ACCEL_XOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_XOUT_L);
    float ay = (readMPU6050(MPU6050_ACCEL_YOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_YOUT_L);
    float az = (readMPU6050(MPU6050_ACCEL_ZOUT_H) << 8) | readMPU6050(MPU6050_ACCEL_ZOUT_L);
    float degRoll = atan2(ay, az) * RAD_TO_DEG;
    float degPitch = atan(-ax / sqrt(ay * ay + az * az)) * RAD_TO_DEG;

    // ジャイロで角速度を求める。
    float gx = (readMPU6050(MPU6050_GYRO_XOUT_H) << 8) | readMPU6050(MPU6050_GYRO_XOUT_L);
    float gy = (readMPU6050(MPU6050_GYRO_YOUT_H) << 8) | readMPU6050(MPU6050_GYRO_YOUT_L);
    float gz = (readMPU6050(MPU6050_GYRO_ZOUT_H) << 8) | readMPU6050(MPU6050_GYRO_ZOUT_L);
    float dpsX = gx / 131.0; // LSB sensitivity: 131 LSB/dps @ ±250dps
    float dpsY = gy / 131.0;
    float dpsZ = gz / 131.0;

    // カルマンフィルタで角度(x,y)を計算する。
    long curMicros = micros();
    float dt = (float)(curMicros - gPrevMicros) / 1000000; // μsec -> sec
    gPrevMicros = curMicros;
    float degX = gKfx.calcAngle(degRoll, dpsX, dt);
    float degY = gKfy.calcAngle(degPitch, dpsY, dt);
    degY -= gCalibrateY;

    // PID制御でモーター出力を計算。
    gPowerP = degY / 90;           // P成分：傾き-90～90度 → -1～1
    gPowerI += gPowerP;            // I成分：傾きの積算。
    gPowerD = dpsY / 250;          // D成分：角速度-250～250dps → -1～1
    float power = gPowerP * 17.0 + // この数字は試行錯誤で調整。
                  gPowerI * 1.5 +
                  gPowerD * 2.0;
    power = max(-1, min(1, power)); // → -1～1

    int dir = (-power > 0) ? 0 : 1;
    int val = abs((int)(power * 255));
    int sendVal = abs((int)(power * 100));
    mySerial.write(dir << 7 | map(sendVal, 0, 100, 30, 100));
    /*
    if (abs(gPowerI) > 1)
    {
        if (gPowerI < 0)
            gCalibrateY += 0.1;
        else
            gCalibrateY += 0.1;
    }
    */
    // デバッグ用。
    static int ps;
    if (++ps % 1 == 0)
    {

        Serial.print("Calib:");
        Serial.print(gCalibrateY);
        Serial.print("\tdt:");
        Serial.print(dt);
        Serial.print("\tdegY:");
        Serial.print(degY);
        Serial.print("\tP:");
        Serial.print(gPowerP);
        Serial.print("\tI:");
        Serial.print(gPowerI);
        Serial.print("\tD:");
        Serial.print(gPowerD);
        Serial.print("\tpwr:");
        Serial.print(power);
        Serial.println("");
    }

    delay(3);
}