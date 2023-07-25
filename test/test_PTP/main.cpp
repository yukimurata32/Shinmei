#include <Arduino.h>
#include <TimerThree.h>
#include <Wire.h>
#include <array>
#include <vector>

#include <point_to_point.h>

std::array<float,3> start{{0.0,0.0,0.0}};

std::array<float,3> goal{{0.0,0.0,1000.0}};

std::array<float,3> point{{0.0,0.0,0.0}};

void setup() {
    Serial.begin(115200);
    PTP ptp_;

    while(ArrayMath::distance(goal,point)>1.0f){
        ptp_.point_to_point_profile(start,point, goal,100.0f,1.0f, point);
        Serial.println("test");
        Serial.print(point[0]); Serial.print(",");
        Serial.print(point[1]); Serial.print(",");
        Serial.print(point[2]); Serial.println(",");
    }

    //Serial.println("Hello world!");
}

void loop() {

}