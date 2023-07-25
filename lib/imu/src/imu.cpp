#include "imu.h"

IMU::IMU(const uint32_t baudrate, HardwareSerial& s) : serial_(&s){
    JY901.attach(*serial_);
    serial_->begin(baudrate);
    while(!*serial_);
}

bool IMU::setReturnHz(const CJY901::ReturnHz hz) {
    uint8_t buff_return_rate[5] ={0xFF, 0xAA, CJY901::Reg::ReturnRateReg, hz, 0x00};
    size_t num = serial_->write(buff_return_rate, 5);
    return (num == 5);
}

bool IMU::setBaudrate(const CJY901::BaudRate buadrate) {
    uint8_t buff_baudrate[5] = {0xFF, 0xAA, CJY901::Reg::BaudRateReg, buadrate, 0x00};
    size_t num = serial_->write(buff_baudrate, 5);

    return (num == 5);
}

void IMU::Rx(const float roll, std::array<float,3>& vec) {
    float _cos = std::cos(roll);
    float _sin = std::sin(roll);
    float temp = -vec[1] * _sin + vec[2] * _cos;
    vec[1] = vec[1] * _cos + vec[2] * _sin;
    vec[2] = temp;
}

void IMU::Ry(const float pitch, std::array<float, 3>& vec) {
    float _cos = std::cos(pitch);
    float _sin = std::sin(pitch);
    float temp = vec[0] * _sin + vec[2] * _cos;
    vec[0] = vec[0] * _cos - vec[2] * _sin;
    vec[2] = temp;
}

void IMU::Rz(const float yaw, std::array<float, 3>& vec) {
    float _cos = std::cos(yaw);
    float _sin = std::sin(yaw);
    float temp = -vec[0] * _sin + vec[1] * _cos;
    vec[0] = vec[0] * _cos + vec[1] * _sin;
    vec[1] = temp;
}

void IMU::getAccel(std::array<float, 3>& acc) {
    acc[0] = JY901.getAccX();
    acc[1] = JY901.getAccY();   
    acc[2] = JY901.getAccZ();
    std::array<float, 3> g{{0.0, 0.0, -kGravity_}};
    // grobal to body coordinate system. 
    Ry(pos_[1], g);
    Rx(pos_[0], g);
    for (size_t i = 0; i < 3; ++i) acc[i] -= g[i];
}

bool IMU::getPosture(std::array<float, 3>& pos) {
    pos[0] = static_cast<float>(JY901.getRoll());
    pos[1] = static_cast<float>(JY901.getPitch());
    pos[2] = static_cast<float>(JY901.getYaw());
    for (size_t i = 0; i < 3; ++i){
        if (pos[i] > 180) pos[i] -= 360;
        if (std::abs(pos[i]) > 75 && i != 2) {return false;} // ignore yaw angular
        pos[i] *= kDegToRad_;
    }
    return true;
}

void IMU::getPostureVel(std::array<float, 3>& pos_vel) {
    pos_vel[0] = JY901.getGyroX();
    pos_vel[1] = JY901.getGyroY();
    pos_vel[2] = JY901.getGyroZ();

    for (size_t i = 0; i < 3; ++i) {
        pos_vel[i] *= kDegToRad_;
    }
}

bool IMU::sensing() {
    is_unsafe_ = false;
    unsigned int start = millis();
    while (JY901.receiveSerialData() == false) {
        if (millis() - start > timeout_ && timeout_ > 0) 
            return false;
        // else Serial.println("Timeout");
    }

    if (!getPosture(pos_)) is_unsafe_ = true;
    getPostureVel(pos_vel_);
    pos_[2] = 0.0f;
    pos_vel_[2] = 0.0f;

    getAccel(acc_);
    get_data_ = true;
    return true;
}