#pragma once

#include <Arduino.h>
#include <Dynamixel.h>
#include <array>
#include <array_math.h>
#include <kinematics.h>
#include <leg.h>
#include <memory>
#include <types.h>
#include <vector>

class Motor {
private:
  enum Reg {
    HardwareErrorStatus = 70,
    PresentCurrent = 126,
    PresentVelocity = 128,
    PresentPosition = 132,
    PresentTempreture = 146
  };
  enum ErrorStatus {
    ExceedVoltage = 0x01,
    ExceedTemperature = 0x04,
    EncoderError = 0x08,
    ElectricalShockError = 0x10,
    OverloadError = 0x20
  };
  enum DriveMode {
    Normal = 0x00,
    Reverse = 0x01,
  };
  const uint8_t PIN_RTS1_ = 2;
  const uint8_t PIN_RTS2_ = 12;
  const uint8_t PIN_RTS3_ = 11;
  const uint8_t PIN_RTS4_ = 26;

  std::array<std::shared_ptr<Dynamixel>, 2> dxif;

  int operating_mode_ = 3; // OperatingMode

public:
  enum OperatingMode {
    CurrentControl = 0,
    VelocityControl = 1,
    PositionControl = 3,
    ExetendedPositionControl = 4,
    CurrentBasePositionControl = 5,
    PWMControl = 16
  };
  uint8_t channel_[6] = {0, 0, 0, 1, 1, 1};

  uint8_t ID_[6][3] = {{41, 42, 43}, {51, 52, 53}, {61, 62, 63},
                       {31, 32, 33}, {21, 22, 23}, {11, 12, 13}};
  int pm_[6][3] = {{-1, -1, -1}, {-1, -1, -1}, {-1, -1, -1},
                   {-1, 1, 1},   {-1, 1, 1},   {-1, 1, 1}};

  // origin1 = 0.0; origin2 = 92.55, origin3 = 56.45
  // uint16_t origin[6][3] = {
  //     {2048-70,  2048+512+40, 2048 - 150},
  //     {2048+200, 2048+512+20, 2048+5},
  //     {2048+50,  2048+512+3,  2048+45},
  //     {2048-25,  2048-512-20, 2048-10},
  //     {2048-40,  2048-512-40, 2048-20},
  //     {2048+500, 2048-512-10, 2048-40}
  // };

  // uint16_t origin[6][3] = {
  //     {2048 - origin_1, 2048 + origin_2 + 40, 2048 + origin_3},
  //     {2048 - origin_1 + 200, 2048 + origin_2 + 20, 2048 + origin_3 - 50},
  //     {2048 - origin_1 + 50, 2048 + origin_2 + 3, 2048 + origin_3 + 45},
  //     {2048 - origin_1 - 25, 2048 - origin_2 - 20, 2048 - origin_3 + 20},
  //     {2048 - origin_1 - 40, 2048 - origin_2 + 100, 2048 - origin_3},
  //     {2048 - origin_1 + 500, 2048 - origin_2 - 10, 2048 - origin_3 - 40}};

  const uint16_t origin_1 = 0.0;
  const uint16_t origin_2 = 512 + 2048+50 /*512.0*/; ////(32.55f + 60.0f) * 2048 / 180;
  const uint16_t origin_3 = -250;             //(90.0f - 33.25f) * 2048 / 180;
  // difference between motor origin and theoretical origin

  /*uint16_t origin[6][3] = {
      {2048 - origin_1-76, 2048 + origin_2+75, 2048 + origin_3- 10},
      {2048 - origin_1-20, 2048 + origin_2- 21+50, 2048 + origin_3+ 38},
      {2048 - origin_1-50, 2048 + origin_2- 36+45, 2048 + origin_3+ 16+50},
      {2048 - origin_1-25, 2048 - origin_2+ 11, 2048 - origin_3- 35},
      {2048 - origin_1+12, 2048 - origin_2+ 45-20, 2048 - origin_3-151-50},
      {2048 - origin_1+ 6, 2048 - origin_2+ 28, 2048 - origin_3- 79}
  };*/

  uint16_t origin[6][3] = {
    {2048 - origin_1-  6, 2048 + origin_2- 1946-150-25, 2048 + origin_3 +188+100},
    {2048 - origin_1-  21, 2048 + origin_2- 1924-150-25, 2048 + origin_3+ 199+100},
    {2048 - origin_1+  107, 2048 + origin_2-  1907-150-25, 2048 + origin_3+ 162+100},
    {2048 - origin_1+  50, 2048 - origin_2+ 2166-150+25, 2048 - origin_3-  83-100},
    {2048 - origin_1+  2, 2048 - origin_2+ 2298-150+25, 2048 - origin_3-296-100},
    {2048 - origin_1+  14, 2048 - origin_2+  2213-150+25, 2048 - origin_3-215-100}
  };
/*uint16_t origin2[6][3] ={2042,1946,188},
                      {2027,1924,191},
                      {2155,1907,162},
                      {2098,2166,4013},
                      {2046,2298,3800},
                      {2034,2213,3881}*/

  // minimum position of motors
  const uint32_t min_position_[6][3] = {
      {2048 - 2048/2, 2048 - 2048 * 3/4, 2048 - 2048 * 172/180},
      {2048 - 2048/2, 2048 - 2048 * 3/4, 2048 - 2048 * 172/180},
      {2048 - 2048/2, 2048 - 2048 * 3/4, 2048 - 2048 * 172/180},
      {2048 - 2048/2, 2048 - 2048 * 50/100, 2048 - 2048/10},
      {2048 - 2048/2, 2048 - 2048 * 50/100, 2048 - 2048/10},
      {2048 - 2048/2, 2048 - 2048 * 45/100, 2048 - 2048/10}
  };

  // maxmum position of motors
  const uint32_t max_position_[6][3] = {
      {2048 + 2048/2, 2048 + 2048 * 45/100, 2048 + 2048/10},
      {2048 + 2048/2, 2048 + 2048 * 50/100, 2048 + 2048/10},
      {2048 + 2048/2, 2048 + 2048 * 50/100, 2048 + 2048/10}, 
      {2048 + 2048/2, 2048 + 2048 * 3/4, 2048 + 2048 * 172/180},
      {2048 + 2048/2, 2048 + 2048 * 3/4, 2048 + 2048 * 172/180},
      {2048 + 2048/2, 2048 + 2048 * 3/4, 2048 + 2048 * 172/180}
  };

  // 0.229 * value [rpm] = 0.229 * value * 2PI / 60.0 [rad/sec]
  const float kDynamixel2Vel_ = 0.229f * 0.5 * M_PI / 60.0f;
  const float kVel2Dynamixel_ = 1.0f / kDynamixel2Vel_;
  // 2.69 * value [mA] -> 2.69 * 0.002 [N*m / mA]
  const float kCurrent_ = 2.69 * 0.002;

private:
  int32_t RadToDynamixel(const float src, const uint16_t origin, const int pm) {
    int32_t dist = static_cast<uint32_t>(pm * src * _PI2048 + origin);
    while (dist < 0)
      dist += 4095;
    while (dist > 4095)
      dist -= 4095;
    return dist;
  }

  float DynamixelToRad(const int32_t src, const uint16_t origin, const int pm) {
    return pm * static_cast<float>((src - origin) * PI2048);
  }

  int32_t Rad2DynamixelVel(const float src, const int pm) {
    return src * kVel2Dynamixel_ * ArrayMath::sign(pm);
  }

  float Dynamixel2RadVel(const float src, const int pm) {
    return src * kDynamixel2Vel_ * ArrayMath::sign(pm);
  }

public:
  ~Motor() {
    Serial3.end();
    Serial4.end();
  }
  void setup(const long baudrate) {
    dxif[0] = std::make_shared<Dynamixel>(PIN_RTS3_);
    dxif[1] = std::make_shared<Dynamixel>(PIN_RTS4_);
    Serial3.begin(baudrate);
    Serial4.begin(baudrate);

    while (!Serial3)
      ;
    while (!Serial4)
      ;

    dxif[0]->attach(Serial3, baudrate);
    dxif[1]->attach(Serial4, baudrate);

    for (int i = 0; i < 6; ++i) {
      for (unsigned int j = 0; j < 3; ++j) {
        const uint8_t id = ID_[i][j];
        dxif[channel_[i]]->addModel<DxlModel::X>(id);
        dxif[channel_[i]]->minPositionLimit(id, min_position_[i][j]);
        dxif[channel_[i]]->maxPositionLimit(id, max_position_[i][j]);
        dxif[channel_[i]]->velocityLimit(id, 1023);
        dxif[channel_[i]]->accelerationLimit(id, 32767);
        // dxif[channel_[i]]->torqueEnable(id, enable);
      }
    }
  }

  void enableTorque() {
    for (int i = 0; i < 6; ++i)
      for (int j = 0; j < 3; ++j)
        dxif[channel_[i]]->torqueEnable(ID_[i][j], true);
  }

  void setEnableTorque(Leg& leg, int joint,bool enable){
            dxif[channel_[leg.getID()]]->torqueEnable(ID_[leg.getID()][joint], enable);

  }
  /*void setEnableTorque(Leg& leg,std::array<bool,3>& enable){
      for(int i = 0; i < 3; ++i) dxif[channel_[leg.getID()]]->torqueEnable(ID_[leg.getID()][i], enable[i]);
  }*/
  void setEnableTorque(Leg& leg,bool enable){
      for(int i = 0; i < 3; ++i) dxif[channel_[leg.getID()]]->torqueEnable(ID_[leg.getID()][i], enable);
      //std::array<bool,3> enables; enables.fill(enable);
      //dxif[channel_[leg.getID()]]->syncwritetorqueEnable(ID_[leg.getID()], enables,3);
  }

  bool setOperatingMode(const OperatingMode mode) {
    /* set modes */
    uint8_t modes[9];
    for (int i = 0; i < 9; ++i)
      modes[i] = mode;
    /* set ids */
    for (int i = 0; i < 2; ++i) {
      uint8_t ids[9];
      int num = 0;

      for (int j = 0; j < 6; ++j) {
        if (channel_[j] == i) {
          for (int k = 0; k < 3; ++k)
            ids[num * 3 + k] = ID_[j][k];
        }
      }
      if (num != 3)
        return false;
      /* try until success */
      while (!dxif[i]->syncwriteoperatingMode(ids, modes, 9))
        delay(100);
    }
    operating_mode_ = mode;
    return true;
  }

  bool setPosition(float rad, int leg_id, int joint) {
    float input =
        RadToDynamixel(rad, origin[leg_id][joint], pm_[leg_id][joint]);
    return dxif[channel_[leg_id]]->goalPosition(ID_[leg_id][joint], input);
  }
  bool setVelocity(float vel, int leg_id, int joint) {
    float input = kVel2Dynamixel_ * vel;
    return dxif[channel_[leg_id]]->goalVelocity(ID_[leg_id][joint], input);
  }

  bool setPositionPGain(uint16_t gain, int leg_id, int joint) {
    return dxif[channel_[leg_id]]->positionPGain(ID_[leg_id][joint], gain);
  }
  bool setPositionIGain(uint16_t gain, int leg_id, int joint) {
    return dxif[channel_[leg_id]]->positionIGain(ID_[leg_id][joint], gain);
  }
  bool setPositionDGain(uint16_t gain, int leg_id, int joint) {
    return dxif[channel_[leg_id]]->positionDGain(ID_[leg_id][joint], gain);
  }
  bool setVelocityPGain(uint16_t gain, int leg_id, int joint) {
    return dxif[channel_[leg_id]]->velocityPGain(ID_[leg_id][joint], gain);
  }
  bool setVelocityIGain(uint16_t gain, int leg_id, int joint) {
    return dxif[channel_[leg_id]]->velocityIGain(ID_[leg_id][joint], gain);
  }
  void setVelocity(uint32_t speed) {
    uint32_t s[3] = {speed, speed, speed};
    for (int i = 0; i < 6; ++i)
      dxif[channel_[i]]->syncwriteprofileVelocity(ID_[i], s, 3);
  }

  void setAccel(uint32_t accel) {
    uint32_t a[3] = {accel, accel, accel};
    for (int i = 0; i < 6; ++i)
      dxif[channel_[i]]->syncwriteprofileAcceleration(ID_[i], a, 3);
  }

  void setPositionPGain(uint16_t gain) {
    uint16_t g[3] = {gain, gain, gain};
    for (int i = 0; i < 6; ++i)
      dxif[channel_[i]]->syncwritepositionPGain(ID_[i], g, 3);
  }

  void setPositionIGain(uint16_t gain) {
    uint16_t g[3] = {gain, gain, gain};
    for (int i = 0; i < 6; ++i)
      dxif[channel_[i]]->syncwritepositionIGain(ID_[i], g, 3);
  }

  void setPositionDGain(uint16_t gain) {
    uint16_t g[3] = {gain, gain, gain};
    for (int i = 0; i < 6; ++i)
      dxif[channel_[i]]->syncwritepositionDGain(ID_[i], g, 3);
  }

  void setVelocityPGain(uint16_t gain) {
    uint16_t g[3] = {gain, gain, gain};
    for (int i = 0; i < 6; ++i)
      dxif[channel_[i]]->syncwritevelocityPGain(ID_[i], g, 3);
  }

  void setVelocityIGain(uint16_t gain) {
    uint16_t g[3] = {gain, gain, gain};
    for (int i = 0; i < 6; ++i)
      dxif[channel_[i]]->syncwritevelocityIGain(ID_[i], g, 3);
  }

  bool setSyncPosition(Leg &leg) {
    const int id = leg.getID();
    int32_t output_angle[3];
    for (int i = 0; i < 3; ++i)
      output_angle[i] =
          RadToDynamixel(leg.getAngle()[i], origin[id][i], pm_[id][i]);
    return dxif[channel_[id]]->syncwritegoalPosition(ID_[id], output_angle, 3);
  }

  void setSyncAllPosition(std::array<Leg, 6> &legs) {
    for (int i = 0; i < 2; ++i) {
      int num = 0;
      int32_t output_angle[9];
      uint8_t ids[9];
      for (Leg &leg : legs) {
        // bool flag = false;
        if (channel_[leg.getID()] == i) {
          for (int j = 0; j < 3; ++j) {
            ids[3 * num + j] = ID_[leg.getID()][j];
            output_angle[3 * num + j] = RadToDynamixel(
                leg.getAngle()[j], origin[leg.getID()][j], pm_[leg.getID()][j]);
            // output_angle[3 * num + j] = leg.angle_joint_[j] * _PI2048 +
            // origin[leg.getID()][j];
          }
          ++num;
        }
        // if (flag) Kinematics::FwdKinematics(leg.angle_joint_,
        // leg.getTipLpos(), leg);
      }
      if (num != 3)
        continue;
      dxif[i]->syncwritegoalPosition(ids, output_angle, 9);
    }
  }

  bool setSyncAllPosition(std::array<std::array<float, 3>, 6> &angles) {
    for (uint8_t i = 0; i < 2; ++i) {
      uint8_t num = 0;
      int32_t output_angle[9];
      uint8_t ids[9];
      for (uint8_t j = 0; j < 6; ++j) {
        if (channel_[j] == i) {
          for (uint8_t k = 0; k < 3; ++k) {
            ids[3 * num + k] = ID_[j][k];
            output_angle[3 * num + k] =
                RadToDynamixel(angles[j][k], origin[j][k], pm_[j][k]);
          }
          ++num;
        }
      }
      if (num != 3)
        continue;
      bool result = dxif[i]->syncwritegoalPosition(ids, output_angle, 9);
      if (!result)
        return false;
    }
    return true;
  }

  bool setSyncAllVelocities(std::array<std::array<float, 3>, 6> &vels) {
    uint8_t num = 0;
    int32_t output_vels[9];
    uint8_t ids[9];
    for (uint8_t i = 0; i < 2; ++i) {
      for (uint8_t j = 0; j < 6; ++j) {
        if (channel_[j] == i) {
          for (uint8_t k = 0; k < 3; ++k) {
            ids[3 * num + k] = ID_[j][k];
            output_vels[3 * num + k] = Rad2DynamixelVel(vels[j][k], pm_[j][k]);
          }
          ++num;
        }
      }
      if (num != 3)
        continue;
      bool result = dxif[i]->syncwritegoalVelocity(ids, output_vels, 9);
      if (!result)
        return false;
    }
    return true;
  }

  float getPosition(int leg_id, int joint) {
    int32_t output;
    output = dxif[channel_[leg_id]]->presentPosition(ID_[leg_id][joint]);
    if (output == 0xFFFFFFFF)
      return 0xFFFFFFFF;
    return DynamixelToRad(output, origin[leg_id][joint], pm_[leg_id][joint]);
  }

  void getPosition(Leg &leg, std::array<float, 3> &dist) {
    int32_t joint_get[3];
    const int id = leg.getID();
    // dxif[channel_[id]].syncreadpresentPosition(ID_[id], joint_get, 3);
    for (size_t i = 0; i < 3; ++i)
      joint_get[i] = dxif[channel_[id]]->presentPosition(ID_[id][i]);
    for (int i = 0; i < 3; ++i)
      dist[i] = static_cast<float>(joint_get[i] - origin[id][i]) * PI2048;
  }

  float getVelocity(int leg_id, int joint) {
    int32_t output = dxif[channel_[leg_id]]->goalVelocity(ID_[leg_id][joint]);
    if (output == 0xFFFFFFFF)
      return 0xFFFFFFFF;
    return Dynamixel2RadVel(output, pm_[leg_id][joint]);
  }

  void getSyncAllPosition(std::array<Leg, 6> &legs, const bool observed) {
    int num = 0;
    int leg_id[3] = {0, 0, 0};
    int32_t output[9];
    uint8_t ids[9];
    std::array<std::array<float, 3>, 3> get_angle;
    for (int i = 0; i < 2; ++i) {
      num = 0;
      for (const Leg &leg : legs) {
        if (channel_[leg.getID()] == i) {
          for (int j = 0; j < 3; ++j)
            ids[3 * num + j] = ID_[leg.getID()][j];
          leg_id[num] = leg.getID();
          ++num;
        }
      }
      if (num != 3)
        continue;
      dxif[i]->syncreadpresentPosition(ids, output, 9);
      int32_t output1[3] = {output[0], output[1], output[2]};
      int32_t output2[3] = {output[3], output[4], output[5]};
      int32_t output3[3] = {output[6], output[7], output[8]};

      for (int k = 0; k < 3; ++k) {
        get_angle[0][k] =
            DynamixelToRad(output1[k], origin[leg_id[0]][k], pm_[leg_id[0]][k]);
        get_angle[1][k] =
            DynamixelToRad(output2[k], origin[leg_id[1]][k], pm_[leg_id[1]][k]);
        get_angle[2][k] =
            DynamixelToRad(output3[k], origin[leg_id[2]][k], pm_[leg_id[2]][k]);
      }
      if (observed == true) {
        legs[leg_id[0]].setObsAngle(get_angle[0]);
        legs[leg_id[1]].setObsAngle(get_angle[1]);
        legs[leg_id[2]].setObsAngle(get_angle[2]);
      } else {
        legs[leg_id[0]].setAngle(get_angle[0]);
        legs[leg_id[1]].setAngle(get_angle[1]);
        legs[leg_id[2]].setAngle(get_angle[2]);
      }
      Serial.print("angle ");
      Serial.print(i);
      Serial.print(" ");
      for (int ii = 0; ii < 3; ++ii) {
        for (int jj = 0; jj < 3; ++jj) {
          Serial.print(get_angle[ii][jj]);
          Serial.print(" ");
        }
      }
      Serial.println();
    }
  }

  void getSyncAllPosition(std::array<std::array<float, 3>, 6> &angles) {
    for (int i = 0; i < 2; ++i) {
      int num = 0;
      int leg_id[3] = {0, 0, 0};
      int32_t output[9];
      uint8_t ids[9];
      for (uint8_t j = 0; j < 6; ++j) {
        if (channel_[j] == i) {
          for (uint8_t k = 0; k < 3; ++k)
            ids[3 * num + k] = ID_[j][k];
          leg_id[num] = j;
          ++num;
        }
      }
      if (num != 3)
        continue;
      dxif[i]->syncreadpresentPosition(ids, output, 9);
      int32_t output1[3] = {output[0], output[1], output[2]};
      int32_t output2[3] = {output[3], output[4], output[5]};
      int32_t output3[3] = {output[6], output[7], output[8]};
      for (uint8_t k = 0; k < 3; ++k) {
        angles[leg_id[0]][k] =
            DynamixelToRad(output1[k], origin[leg_id[0]][k], pm_[leg_id[0]][k]);
        angles[leg_id[1]][k] =
            DynamixelToRad(output2[k], origin[leg_id[1]][k], pm_[leg_id[1]][k]);
        angles[leg_id[2]][k] =
            DynamixelToRad(output3[k], origin[leg_id[2]][k], pm_[leg_id[2]][k]);
      }
    }
  }

  int getPosVelTorque(std::array<std::array<float, 3>, 6> &pos,
                      std::array<std::array<float, 3>, 6> &vel,
                      std::array<std::array<float, 3>, 6> &torque) {
    for (size_t i = 0; i < 2; ++i) {
      int num = 0;
      int leg_id[3] = {0, 0, 0};
      uint32_t output[27];
      uint8_t ids[9];
      uint16_t sbyte[3] = {2, 4, 4};
      for (uint8_t j = 0; j < 6; ++j) {
        if (channel_[j] == i) {
          for (uint8_t k = 0; k < 3; ++k)
            ids[3 * num + k] = ID_[j][k];
          leg_id[num] = j;
          ++num;
        }
      }
      if (num != 3)
        return 255;
      int result = dxif[i]->syncreadmultipledata(ids, output, PresentCurrent,
                                                 sbyte, 3, 9);
      if (result >= 0)
        return result;
      for (uint8_t j = 0; j < 3; ++j) {
        uint8_t id = leg_id[j];
        int ind = 9 * j;
        for (uint8_t k = 0; k < 3; ++k)
          torque[id][k] = (int16_t)output[ind + k * 3] * kCurrent_ * pm_[id][k];
        for (uint8_t k = 0; k < 3; ++k)
          vel[id][k] =
              (int32_t)output[ind + k * 3 + 1] * kDynamixel2Vel_ * pm_[id][k];
        for (uint8_t k = 0; k < 3; ++k)
          pos[id][k] = DynamixelToRad((int32_t)output[ind + k * 3 + 2],
                                      origin[id][k], pm_[id][k]);
      }
      delayMicroseconds(10);
    }
    return -1;
  }

  inline int getErrorStatus(std::array<std::array<int, 3>, 6> &error) {
    for (size_t i = 0; i < 2; ++i) {
      int num = 0;
      int leg_id[3] = {0, 0, 0};
      uint32_t output[9];
      uint8_t ids[9];
      uint16_t sbyte[1] = {1};
      for (uint8_t j = 0; j < 6; ++j) {
        if (channel_[j] == i) {
          for (uint8_t k = 0; k < 3; ++k)
            ids[3 * num + k] = ID_[j][k];
          leg_id[num] = j;
          ++num;
        }
      }
      if (num != 3)
        return 255;
      int result = dxif[i]->syncreadmultipledata(
          ids, output, Reg::HardwareErrorStatus, sbyte, 1, 9);
      if (result >= 0)
        return result;
      for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k)
          error[leg_id[j]][k] = output[3 * j + k];
    }
    return -1;
  }

  int getPresentTempreture(std::array<std::array<float, 3>, 6> &heats) {
    for (size_t i = 0; i < 2; ++i) {
      int num = 0;
      int leg_id[3] = {0, 0, 0};
      uint32_t output[9];
      uint8_t ids[9];
      uint16_t sbyte[3] = {2, 4, 4};
      for (uint8_t j = 0; j < 6; ++j) {
        if (channel_[j] == i) {
          for (uint8_t k = 0; k < 3; ++k)
            ids[3 * num + k] = ID_[j][k];
          leg_id[num] = j;
          ++num;
        }
      }
      if (num != 3)
        return 255;
      int result = dxif[i]->syncreadmultipledata(ids, output, PresentTempreture,
                                                 sbyte, 1, 9);
      if (result >= 0)
        return result;
      for (int j = 0; j < 3; ++j)
        for (int k = 0; k < 3; ++k)
          heats[leg_id[j]][k] = output[3 * j + k];
    }
    return -1;
  }

  inline int getOperatingMode() const { return operating_mode_; }

  void getAllHomingOffset(){
    //std::array<std::array<float, 3>, 6> offset;
    for(int i=0;i<6;i++){
      Serial.print("{");
      for(int j=0;j<3;j++){
        //int32_t output = dxif[channel_[i]]->presentPosition(ID_[i][j]);
        //dxif[channel_[i]]->homingOffset(ID_[i][j], output);
        
        //Serial.print(i); Serial.print(",");Serial.print(j); Serial.print(",");
        float output = dxif[channel_[i]]->presentPosition(ID_[i][j]);
        if(j==0){
          Serial.print(output - (2048-origin_1));
        }else{
          Serial.print(output);
        }
        if(j<2){Serial.print(",");}
        //Serial.println(dxif[channel_[i]]->homingOffset(ID_[i][j]));
      }
      Serial.println("},");
    }
  }
};


 /*uint16_t origin[6][3] = {
    {2048 - origin_1-  6, 2048 + origin_2- 1946-150, 2048 + origin_3 +188},
    {2048 - origin_1-  21, 2048 + origin_2- 1924-150, 2048 + origin_3+ 199},
    {2048 - origin_1+  107, 2048 + origin_2-  1907-150, 2048 + origin_3+ 162},
    {2048 - origin_1+  50, 2048 - origin_2+ 2166-150, 2048 - origin_3-  83},
    {2048 - origin_1-  2, 2048 - origin_2+ 2298-150, 2048 - origin_3-296},
    {2048 - origin_1-  14, 2048 - origin_2+  2213-150, 2048 - origin_3-215}
  };
  uint16_t origin2[6][3] ={2042,1946,188},
                      {2027,1924,191},
                      {2155,1907,162},
                      {2098,2166,4013},
                      {2046,2298,3800},
                      {2034,2213,3881}*/