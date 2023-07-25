# IMU Class

# 役割
JY901の情報を取ってきて，姿勢角をラジアン[rad]に変換する. 

# 使い方
センサとマイコンが繋がっているシリアルとボードレートを指定してセットアップを行う．
```cpp
#include <imu.h>
#include <robot.h>
/* ボードレート: 230400, シリアル 5 */
IMU imu(230400, Serial5)

#define RADIUS 81.068
#define LINK_LENGTH {{60.405, 10.50, 100.0, 146.7}}
#define INCLINATION M_PI / 6
#define BODY_HEIGHT 160
Robot robot(RADIUS, LINK_LENGTH, INCLINATION);

void setup() {
    Serial.begin(9600);
    while(!Serial);
    imu.sensing(robot); // 姿勢角取得

    /* ロボットとIMUの両方に保存される */
    Serial.print("Robot : "); 
    for (uint8_t i = 0; i < 3; ++i) {
        Serial.print(robot.getPosture()[i]);
        Serial.print(", ");
    }
    Serial.pritln();

    Serial.print("IMU : ");
    for (uint8_t i = 0; i < 3; ++i) {
        Serial.print(imu.pos_[i]);
        Serial.print(", ");
    }
}

void loop() {}
```

# Private variables
```cpp
const float kGravity_ = 9.8f // 重力
HardwareSerial* const serial_; // シリアルポート
```

# Public Variables
```cpp
bool get_data_ = true; // データを新しく取得すると trueになる. メインプログラムで値を参照したときに falseに落とす
std::array<float, 3> acc_; // 加速度 [m/s^2] 
std::array<float, 3> pos_; // 姿勢角 [rad]
std::array<float, 3> pos_vel_; // 姿勢角速度 [rad/s]
```

# Private Functions
```cpp
/**
 * @brief sensing()から呼び出す関数．
 *        呼び出す順番が決まっているのと，シリアル通信の場合最新のデータを
 *        まとめてとってくるので，一つの関数にまとめた方が効率が良いため，
 *        Privateにしている．
 */ 
void getAccel(std::array<float, 3>& acc);
bool getPosture(std::array<float, 3>& pos); // 姿勢が大きく水平からずれていると，falseを返す．
void getPostureVel(std::array<float, 3>& pos_vel);
```

# Public Functions
```cpp
IMU(const unsigned long baudrate, HardwareSerial& s); // コンストラクタ

/**
 * @brief センサから最新の姿勢角，姿勢角速度，加速度を取得する　
 * @return true: 平時
 * @return false: 姿勢が大きく水平からずれている
 */ 
bool sensing(Robot& robot); // ロボットへの代入まで行う．
bool sensing(Threads::Mutex& lock); // 排他制御を行い，メンバ変数の更新のみ行う．
```