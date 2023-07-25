#include <Arduino.h>
#include <string.h>
#include <imu.h>

IMU imu(230400, Serial5);

void print3Comp(const String s, const std::array<float, 3>& c) {
    Serial.print(s);
    for (int i = 0; i < 3; ++i) {
        Serial.print(c[i]); 
        Serial.print(" "); 
    }
    Serial.println();
}

void setup() {
    Serial.begin(115200);
    imu.timeout_ = 0; // milliseconds

    // delay(100);
    // imu.setBaudrate(CJY901::BaudRate::_230400);
    // imu.setReturnHz(CJY901::ReturnHz::_200);
    Serial.println("Hello world!");
}

void loop() {
    delay(200);
    if (!imu.sensing()) {
        Serial.println("Faild to receive the sensory data");
    }
    else {
        print3Comp("Angle: ", imu.pos_);
        print3Comp("Angle vel: ", imu.pos_vel_);
        print3Comp("Accelelation: ", imu.acc_);
    }
}