#pragma once

#include <Arduino.h>
#include <array>
#include <JY901.h>
#include <TeensyThreads.h>
#include <cmath>

class IMU {
    private:
        const float kGravity_ = 9.80665f;
        const float kDegToRad_ = M_PI/180.0f;
        HardwareSerial* const serial_;
        
    public:
        bool is_unsafe_ = false;
        unsigned int timeout_ = 0; // milliseconds
        bool get_data_ = true;
        std::array<float, 3> acc_;
        std::array<float, 3> pos_;
        std::array<float, 3> pos_vel_;
        
    private:
        void getAccel(std::array<float, 3>& acc);
        bool getPosture(std::array<float, 3>& pos);
        void getPostureVel(std::array<float, 3>& pos_vel);
        void Rx(const float roll, std::array<float, 3>& vec);
        void Ry(const float pitch, std::array<float, 3>& vec);
        void Rz(const float yaw, std::array<float, 3>& vec);
    
    public:
        IMU(const uint32_t baudrate, HardwareSerial& s);
        bool setReturnHz(const CJY901::ReturnHz hz);
        bool setBaudrate(const CJY901::BaudRate baudrate);
        bool sensing();
};