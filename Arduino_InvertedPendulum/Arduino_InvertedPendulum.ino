#include <SoftwareSerial.h>
#include "IMU.h"
#include "KalmanFilter.h"

// 各項の係数の定義
constexpr double KP = 0.189; // 比例項
constexpr double KI = 1.667; // 積分項
constexpr double KD = 0.010; // 微分項

// 各種変数の宣言
uint32_t pre_time;          // システム開始から前回ループまでの経過時間[usec]
double dt;                  // ループの間隔[sec]
double angle, gyro;         // ロボットの角度と角速度
double calib_angle;         // 角度のキャリブレーション
double integral = 0;        // 角度の積算
double P_val, I_val, D_val; // 各項の出力
double power;               // 最終的な出力値

// 各種インスタンスの生成
KalmanFilter kf;
SoftwareSerial mySerial(A1, A0); // RX, TX

void setup()
{
    // キャリブレーションの変更ボタンの初期化
    pinMode(11, INPUT_PULLUP);
    pinMode(12, INPUT_PULLUP);

    // ハードウェアシリアル：デバッグ用
    Serial.begin(115200);
    // ソフトウェアシリアル：PICとの通信用
    mySerial.begin(57600); // 上限が57600bps

    // IMU初期化
    initIMU();

    // 起動時の角度を加速度計からとる
    angle = calcAccelAngle();
    // カルマンフィルタの初期値に
    kf.setAngle(angle);
    // 角度のキャリブレーションにも
    calib_angle = angle;

    // ループ測定用に記録
    pre_time = micros();
}

void loop()
{
    adjust();  // キャリブレーションの変更
    observe(); // センサの値を取得
    estimat(); // センサの値から機体の姿勢を推定
    plan();    // 車輪の駆動計画を行う
    command(); // PICに車輪の回転数を司令
    debug();   // 各変数の値をシリアルモニタに表示
    delay(1);  // 一応入れておく
}

// functions
/////////////////////////////////////////////////////////////////////////////
double calcAccelAngle()
{
    double a[3];
    for (int i = 0; i < 3; i++)
        a[i] = getAccel(i);

    // 重力加速度（Z方向の加速度）を利用して角度を算出
    return atan(-a[X] / sqrt(a[Y] * a[Y] + a[Z] * a[Z])) * RAD_TO_DEG;
}

void initIMU()
{
    // IMUに起動コマンドを送る
    Wire.begin();
    if (readIMU(IMU_WHO_AM_I) != 0x68)
    {
        Serial.println("IMU ERROR");
        while (true)
            ;
    }

    // 各センサの設定と分解能を指定する
    writeIMU(IMU_SMPLRT_DIV, 0x07);
    writeIMU(IMU_CONFIG, 0x00);
    writeIMU(IMU_GYRO_CONFIG, 0x00);
    writeIMU(IMU_ACCEL_CONFIG, 0x00);
    writeIMU(IMU_PWR_MGMT_1, 0x01);
}

void adjust()
{
    // 押されたボタンに応じてキャリブレーションを変更する
    if (digitalRead(11) == LOW)
        calib_angle -= 0.01;

    if (digitalRead(12) == LOW)
        calib_angle += 0.01;
}

void observe()
{
    angle = calcAccelAngle(); // 加速度計から角度を仮計算
    gyro = getGyro(Y);        // 角速度も測定しておく
}

void estimat()
{
    // カルマンフィルタと角度の積算で使用するループ時間を算出
    uint32_t cur_time = micros();
    dt = (double)(cur_time - pre_time) / 1000000;
    pre_time = cur_time;

    // 角度を算出し，キャリブレーションを引く
    angle = kf.calcAngle(angle, gyro, dt) - calib_angle;
}

void plan()
{
    integral += angle * dt;         // 角度を積算
    P_val = KP * angle;             // 角度に比例
    I_val = KI * integral;          // 角度の時間積分値を使用
    D_val = KD * gyro;              // 角度の微分はジャイロの値を使用
    power = P_val + I_val + D_val;  // 各項の総和をとる
    power = max(-1, min(1, power)); // 値のガード
}

void command()
{
    int dir = (power >= 0) ? 0 : 1;    // CW → 0, CCW → 1
    int val = abs((int)(power * 100)); // %に変換
    val = map(val, 0, 100, 30, 100);   // duty<30%だとモータが回らない為
    val = constrain(val, 0, 100);      // 値のガード

    // 送信データ
    // MSB      → モータの回転方向
    // 下位7bit → 出力値（PWMのduty）
    mySerial.write(dir << 7 | val);
}

void debug()
{
    static int ps;
    if (++ps % 1 == 0)
    {
        Serial.print("Calib:");
        Serial.print(calib_angle);
        Serial.print("\tdt:");
        Serial.print(dt, 6);
        Serial.print("\tangle:");
        Serial.print(angle);
        Serial.print("\tP:");
        Serial.print(P_val);
        Serial.print("\tI:");
        Serial.print(I_val);
        Serial.print("\tD:");
        Serial.print(D_val);
        Serial.print("\tpwr:");
        Serial.print(power);
        Serial.println();
    }
}
