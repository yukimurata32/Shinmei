#include <Arduino.h>
#include <TimerThree.h>
#include <Wire.h>
#include <array>
#include <vector>

/* original libraries */
#include <imu.h>
#include <motor2.h>
#include <rmpflow2_param.h>
#include <robot.h>

#include <PINs.h>
#include <properties.h>

#include <ros_fcp_connector.h>
#include <stdio.h>

#include <add_math.h>
#include <preparation.h>

void AdjustingLegs(Robot& r, Motor& m);

namespace Termination {

void blinkLED(float sec) {
  while (1) {
    unsigned long blink_time = millis();
    if (Global_vars::orange_led)
      digitalWrite(ORANGE_LED, HIGH);
    else
      digitalWrite(ORANGE_LED, LOW);
    while (millis() - blink_time < sec * 1e3)
      ;
    Global_vars::orange_led = !Global_vars::orange_led;
  }
}
void termination(float sec) {
  Timer3.stop();
  blinkLED(sec);
}
} // namespace Termination

/* for reading sensory information and writing target values */
namespace Sensors {

void setupIMU(IMU &imu) {
  imu.timeout_ =
      0; // [milliseconds], 0 means to wait until new sensory data recieved
  imu.is_unsafe_ = false;
}
bool readIMU(IMU &imu, Robot &r) {
  if (imu.sensing()) {
    r.setPosture(imu.pos_);
    r.setPostureVel(imu.pos_vel_);
    r.body_.accel_ = imu.acc_;
    return !imu.is_unsafe_;
  } else
    return !imu.is_unsafe_;
}

struct {
  unsigned int last_sync_time_millis = 0;
  std::array<std::array<float, 3>, 6> input;
  std::array<std::array<float, 3>, 6> output_ang;
  std::array<std::array<float, 3>, 6> output_vel;
  std::array<std::array<float, 3>, 6> output_tor;
  std::array<std::array<float, 3>, 6> output_heats;
  std::array<std::array<int, 3>, 6> errors;
} motor_vars;

void setupMotor(Motor &m, const Motor::OperatingMode mode) {
  m.setup(1e6); // baudrate. connect and initialize
  m.setVelocity(1200);
  m.setAccel(50*5);
  m.setPositionPGain(1000);
  m.setPositionIGain(100);
  m.setVelocityPGain(100);  // default: 100
  m.setVelocityIGain(1920); // default: 1920
  m.setOperatingMode(mode); // Position control, Velocity control
  m.enableTorque();
}

bool readMotor(Motor &m, std::array<Leg, 6> &legs) {
  if (!m.getPosVelTorque(motor_vars.output_ang, motor_vars.output_vel,
                         motor_vars.output_tor)/* or
      !m.getPresentTempreture(motor_vars.output_heats)*/) {
    //Serial.println("Failed to read sensory values from motors");
    return false;
  }
  for (Leg &leg : legs) {
    leg.setObsAngle(motor_vars.output_ang[leg.getID()]);
    Kinematics::FwdKinematicsAll(leg, true);
    leg.observed_angular_velocity_ = motor_vars.output_vel[leg.getID()];
    leg.observed_torque_ = motor_vars.output_tor[leg.getID()];
    leg.tip_force_ = Kinematics::ConvertTorqueToForce(leg, true);
    for (int i = 0; i < 3; ++i)
      leg.tip_force_[i] *= 1e3f; // kN -> N
    // Dynamics::ConvertTorqueToBForce(leg.observed_torque_, leg.tip_force_,
    // leg);
  }
  if (m.getOperatingMode() == Motor::OperatingMode::VelocityControl) {
    for (int i = 0; i < 6; ++i)
      legs[i].sync(); // substitute observed values to non observed values
  }
  return true;
}

bool writeMotor(Motor &m, std::array<Leg, 6> &legs) {
  if ((!m.getOperatingMode())== Motor::OperatingMode::VelocityControl) {
    for (int i = 0; i < 6; ++i) {
      if (legs[i].mode_ != 2)
        motor_vars.input[i] = legs[i].angular_velocity_;
      else
        motor_vars.input[i].fill(0.0f);
    }
    if (!m.setSyncAllVelocities(motor_vars.input)) {
      // Serial.println("Failed to write target values of motors.");
      return false;
    }
  } else {
    for (int i = 0; i < 6; ++i)
      motor_vars.input[i] = legs[i].getAngle();
    if (!m.setSyncAllPosition(motor_vars.input)) {
      // Serial.println("Failed to write target values of motors.");
      return false;
    }
  }

  return true;
}
} // namespace Sensors

namespace Config {

void setPlanningType(bool planning_type,Robot &r){
  r.body_.setPlanningType(planning_type);
  r.automaton_.setPlanningType(planning_type);
}

void setupPins() {
  pinMode(CONVERTER_SWITCH, OUTPUT); // Switch of level converter for m-> (3.3V
                                     // <-> 5V) High:ON, Low: OFF
  pinMode(RELAY_VOLTAGE,
          INPUT_PULLUP);         // voltage of relay switch (High:ON, Low;OFF)
  pinMode(RELAY_SWITCH, OUTPUT); // Switch of relay (Low: ON, High: OFF)
  pinMode(ORANGE_LED, OUTPUT);
  digitalWrite(CONVERTER_SWITCH, HIGH);
  delay(100);
  digitalWrite(RELAY_SWITCH, LOW);
  delay(100);
  digitalWrite(ORANGE_LED, Global_vars::orange_led);
}

void setupAutomaton(Robot &r) {
  r.setAutomatonType(Shinmei);
  r.automaton_.setModeforContactPointPlanning(4);
  r.automaton_.setAngularVelocityLimit(ANGLE_VEL_LIMIT);
  r.automaton_.setAngularAccelerationLimit(1.0f * ANGLE_VEL_LIMIT /
                                           (CONTROL_PERIOD * 1e-6f));
  r.automaton_.setEpsilon({{20.0f, 0.0f, 10.0f, 10.0f}});
  r.automaton_.setSpeed(LEG_SPEED);

  Config::setPlanningType(Global_vars::fast_planning_mode,r);
}

/*void setupRMPflow(Robot &r) {
  r.automaton_.setPlanningMethod(PlanningType::RMPFLOW2);

  for (Leg &leg : r.legs_)
    r.automaton_.rmpflow_[leg.getID()].initialize(
        LENGHT_RMPFLOW, RADIUS_RMPFLOW, &leg.obstacles_,
        ANGLE_BETWEEN_CYLINDER12, ANGLE_BETWEEN_CYLINDER23);
  for (RMPflow2::Root2 &rf : r.automaton_.rmpflow_) {
    RMPflow2::setPoliciesParam(rf, ATTRACTOR_TYPE);
    rf.tip_.enableAttractor();
   rf.enableSelfCollisionAvoidance();
  }
}*/

void setupRMPflow_type(Robot &r) {
  r.automaton_.setPlanningMethod(PlanningType::RMPFLOW2);

  for (Leg &leg : r.legs_)
    r.automaton_.rmpflow_[leg.getID()].initialize(
        LENGHT_RMPFLOW, RADIUS_RMPFLOW, &leg.obstacles_,
        ANGLE_BETWEEN_CYLINDER12, ANGLE_BETWEEN_CYLINDER23);
  for (RMPflow2::Root2 &rf : r.automaton_.rmpflow_) {
    RMPflow2::setPoliciesParam(rf,ATTRACTOR_TYPE);
    rf.tip_.enableAttractor();
   rf.enableSelfCollisionAvoidance();
  }
}

std::array<std::array<float, 3>, 6> standby_point{{{320, -275, -BODY_HEIGHT+250},
                                                   {0, -275, -BODY_HEIGHT+250},
                                                   {-320, -275, -BODY_HEIGHT+250},
                                                   {-320, 275, -BODY_HEIGHT+250},
                                                   {0, 275, -BODY_HEIGHT+250},
                                                   {320, 275, -BODY_HEIGHT+250}}};
                                                   
std::array<float, 3> shift{{-150.0f, 0.0, 0.0}};
void setupLegs(Robot &r) {
  for (Leg &leg : r.legs_) {
    leg.angle_watcher_.setTimer(millis);
    Kinematics::ConvertBodyToLeg(standby_point[leg.getID()], leg.standby_point_,
                                 leg);
    for (Leg &leg : r.legs_)
      leg.setShift(shift);
  }
}

void setupBody(Robot &r) {
  r.body_.setRotationOption(Body::Option::EulerRotation);
  r.option_subgoal_ = Body::Observed_OnlyContacted;
  r.option_triangle_ = Body::Observed_OnlyContacted;

  r.setGainPosture(BODY_GAIN_POSTURE);
  r.setGainPostureVel(BODY_GAIN_POSTURE_VEL * CONTROL_PERIOD * 1e-6f);
  r.setSaturationPosture(BODY_SATURATION_POSTURE);
  r.body_.posture_dead_zone_ = BODY_POSTURE_DEADZONE;//PI180/2;

  /* body position control*/
  r.setHeight(BODY_HEIGHT);
  r.setMargin(BODY_STABLE_MARGIN);
  r.setCog(COG);
  r.setSubgoalType(FuncType::Shinmei);

  r.body_.profile_ = true;
  r.setJark(JARK);
  r.setMaxAcc(M_ACC);
  r.setMaxVel(m_VEL);
  r.setProfileIteration(ITERATION);
}

void setupForceControl(Robot &r) {
  r.automaton_.setTakeOverMode(TakeOverMode::Normal);
  r.automaton_.useContactForce();
  r.automaton_.setLoadThresholdInSwingMode(-12.0f);
  r.automaton_.setLoadThresholdInSupportMode(-0.5f);
  r.automaton_.setGainLoadInSupportMode(3.0f);
  //r.automaton_.setGainLoadInSupportMode(0.0f);
}

void setupBehavior(Robot &r) {
  for (uint8_t i = 0; i < 1; ++i) {
    Behavior b_f, b_cw, b_ccw;
    b_f.setDirection(i * M_PI / 3);
    b_cw.setDirection(i * M_PI / 3);
    b_ccw.setDirection(i * M_PI / 3);
    b_f.setType(Behavior::Type::F);
    b_cw.setType(Behavior::Type::CW);
    b_ccw.setType(Behavior::Type::CCW);

    b_f.setFrontList(F_front_list[i]);
    b_f.setRearList(F_rear_list[i]);
    b_f.setOtherSideList(other_side_list[i]);
    b_cw.setFrontList(CW_front_list[i]);
    b_cw.setRearList(CW_rear_list[i]);
    b_cw.setOtherSideList(other_side_list[i]);

    b_ccw.setFrontList(CCW_front_list[i]);
    b_ccw.setRearList(CCW_rear_list[i]);
    b_ccw.setOtherSideList(other_side_list[i]);

    r.bd_.setBehavior(b_f);
    r.bd_.setBehavior(b_cw);
    r.bd_.setBehavior(b_ccw);
  }
}



} // namespace Config

namespace Comm {

std::array<float, 3> preDefinedContactPoint(Leg &leg, float height, float yaw,
                                            const Behavior *b) {
  std::array<float, 3> cp;
  if (b->getType() == Behavior::Type::F) {
    cp[0] = 525.0f; // 575.0f
    cp[1] = 275.0f * leg.getSide();
    cp[2] = -height;
    ArrayMath::Rz(-yaw - b->getRad(), cp);
  } else {
    cp[0] = 400.0f;
    cp[1] = -100.0f * leg.getSide();
    cp[2] = -height;
    if ((b->getType() == Behavior::Type::CW && leg.getSide() == Side::RIGHT) ||
        (b->getType() == Behavior::Type::CCW && leg.getSide() == Side::LEFT)) {
      cp[0] *= -1.0f;
    }
    ArrayMath::Rz(-yaw - b->getRad(), cp);
  }

  return cp;
}
} // end of namespace Comm

/* instances */
Robot robot(RELATIVE_POS, RELATIVE_ANGLE, LINK_LENGTH, INCLINATION, false);
IMU imu(230400, Serial5);
Motor motor;

void Movement() {

  //ros_fcp_converter.receive_.mode_flag=Behavior::Type::CW;
  
  if (Global_vars::communication == ALONE) {
    if (robot.body_.getDistance() < 20.0f) {
      for (Leg &leg : robot.legs_) {
        if (leg.isLeadingLeg() &&
            leg.isRequestingNewCP()) { // search a leg requiring a new contact
                                       // point
          std::array<float, 3> cp;
          cp = Comm::preDefinedContactPoint(leg, robot.getHeight(), 0.0f,
                                            robot.bd_.getBehavior());                            
          robot.setNewCP(leg, cp);
        }
      }
    }
  } else if (Global_vars::communication == ROS) {
    /* Publishする条件？*/
    //RosFcpConnector::publishDebugLog("hz",true);
      
    /* set publish data */
    if(ros_fcp_converter.publish_count==0){
      ros_fcp_converter.setThreeLatestContactPoints(robot);
      ros_fcp_converter.setImu(robot);
      ros_fcp_converter.setPose(robot);
    }
    
    if(Global_vars::publish_motor_data){
      ros_fcp_converter.setMotorAngle(Sensors::motor_vars.output_ang);
      ros_fcp_converter.setMotorEvaluationData(Sensors::motor_vars.output_tor,
                                             Sensors::motor_vars.output_heats);
    }
    ros_fcp_converter.setPlanningLegNumber(robot);
    RosFcpConnector::spin(robot);

    //new_stanby_point
    if(ros_fcp_converter.default_stanby_point_pub_flag_==false) {
      ros_fcp_converter.publish_default_standby_point(Config::standby_point);
    }
    if(ros_fcp_converter.new_stanby_point_flag_==true){
      ros_fcp_converter.update_standby_position(robot);
      ros_fcp_converter.new_stanby_point_flag_=false;
    }



    //bool planning_flag = false;
    if (robot.bd_.getType() == Behavior::Type::F) {
      for (Leg &leg : robot.legs_) {
        if (leg.isLeadingLeg() && leg.isRequestingNewCP()) {
          /*Leg &rear_leg_on_planning_leg_side =
              leg.getFollowedLeg().getFollowedLeg();
          if (rear_leg_on_planning_leg_side.mode_ == 4)
            rear_leg_on_planning_leg_side.deactivateAutomaton();*/
          //if ((Global_vars::fast_planning_mode and robot.body_.getDistance() < 150.0f)  or robot.body_.getDistance() < 20.0f) { // close enough to the subgoal
          //if ((Global_vars::fast_planning_mode and robot.getInTriangle())  or robot.body_.getDistance() < 20.0f) { 
          Leg &rear_leg_on_other_side_leg =leg.getOtherSideLeg().getFollowedLeg().getFollowedLeg();
          if ((Global_vars::fast_planning_mode and 
                  (rear_leg_on_other_side_leg.mode_==1 or
                    rear_leg_on_other_side_leg.mode_==2 or rear_leg_on_other_side_leg .mode_==3 or
                    (rear_leg_on_other_side_leg .isSharingCPwithFollowLeg() and rear_leg_on_other_side_leg .point_>4)))
                      or robot.body_.getDistance() < 20.0f) { 
            
            RosFcpConnector::publishFrontLegNum(); // request a contact point
            
            
                                                  // planning
            //RosFcpConnector::publishDebugLog("pub_triger",Global_vars::debug_mode);
            if (ros_fcp_converter.planning_leg_number_msg_.data >= 0 &&
                ros_fcp_converter.planning_leg_number_msg_.data < 6) {
              Leg &planning_leg =
                  robot.legs_[ros_fcp_converter.planning_leg_number_msg_.data];
              std::array<float, 3> cp = ros_fcp_converter.getContactPoint();
              // cp[2] = -ros_fcp_converter.getHeight();
               //RosFcpConnector::publishDebugLog("planning now",Global_vars::debug_mode);
              if (cp[0] != 0 or cp[1] != 0 or cp[2] != 0) {
                if ((planning_leg.getID() <= 2 and cp[1] < 0) or
                    (planning_leg.getID() >= 3 and cp[1] > 0)) {
                   //RosFcpConnector::publishDebugLog("setnewCP",Global_vars::debug_mode);
                  if(Global_vars::fast_planning_mode){
                    if (robot.setNewCP(planning_leg, cp,subgoal_log_,Global_vars::fast_planning_mode)){
                      //rear_leg_on_planning_leg_side.activateAutomaton();
                      std::array<float, 3> cp_pub;
                      cp_pub[0]=cp[0]-(subgoal_log_[0]-robot.body_.subgoal_[0]);
                      cp_pub[1]=cp[1]-(subgoal_log_[1]-robot.body_.subgoal_[1]);
                      cp_pub[2]=cp[2]-(subgoal_log_[2]-robot.body_.subgoal_[2]);
                      RosFcpConnector::publishNewCP(cp);
                    }
                      

                    /*else {  
                        char msg[10];
                        
                        RosFcpConnector::publishDebugLog("setNewCP false",Global_vars::debug_mode);
                        snprintf(msg,10,"%f",cp[0]-subgoal_log_[0]);
                        RosFcpConnector::publishDebugLog(msg,Global_vars::debug_mode);
                        snprintf(msg,10,"%f",cp[1]-subgoal_log_[1]);
                        RosFcpConnector::publishDebugLog(msg,Global_vars::debug_mode);
                        snprintf(msg,10,"%f",cp[2]-subgoal_log_[2]);
                        RosFcpConnector::publishDebugLog(msg,Global_vars::debug_mode);*/
                        /*snprintf(msg,10,"%f",cp[0]);
                        RosFcpConnector::publishDebugLog(msg,Global_vars::debug_mode);
                        snprintf(msg,10,"%f",subgoal_log_[0]);
                        RosFcpConnector::publishDebugLog(msg,Global_vars::debug_mode);
                        snprintf(msg,10,"%f",subgoal_log_[0]/1000);
                        RosFcpConnector::publishDebugLog(msg,Global_vars::debug_mode);
                    }*/

                  }else{
                    if (robot.setNewCP(planning_leg, cp));
                      //rear_leg_on_planning_leg_side.activateAutomaton();
                  }
                }
              } else {
                 //RosFcpConnector::publishDebugLog("Fill inf",Global_vars::debug_mode);
                cp.fill(1000);
                robot.setNewCP(planning_leg, cp);
              }
            }
            if(ros_fcp_converter.front_leg_num_pub_count_==0){
              for(int i=0;i<3;i++) subgoal_log_[i]=robot.body_.subgoal_[i];
            }
          }
          robot.target_.type_ = ros_fcp_converter.getBehaviorType();
          robot.setHeight(ros_fcp_converter.getHeight());
          robot.target_.posture_ = ros_fcp_converter.getPosture();
          //planning_flag = true;

          if(robot.target_.type_ !=  Behavior::Type::F){
            //RosFcpConnector::publishDebugLog("NOT Behavior::Type::F",Global_vars::debug_mode);
            AdjustingLegs(robot,motor);
            robot.initialize();
            robot.setSubgoalType(Nagoya);
            robot.target_.direction_={std::cos(-60*PI180),std::sin(-60*PI180),0};
            robot.automaton_.setModeforContactPointPlanning(2);
            Config::setPlanningType(false,robot);
          }
          break;
        } else if (leg.isLeadingLeg() && leg.mode_ == 3) {\
          if (Global_vars::use_modification_with_estimated_pose) {
            Leg &planning_leg =
                robot.legs_[ros_fcp_converter.planning_leg_number_msg_.data];
            std::array<float, 3> current_cp = planning_leg.getNewCP();
            std::array<float, 3> cp;
            cp[0] =
                current_cp[0] - ros_fcp_converter.receive_.pose_diff.position.x;
            cp[1] =
                current_cp[1] - ros_fcp_converter.receive_.pose_diff.position.y;
            cp[2] =
                current_cp[2] - ros_fcp_converter.receive_.pose_diff.position.z;

            robot.setNewCP(planning_leg, cp);
          }
        }
      }
    }
    else {
      robot.activateAutomaton();
      robot.target_.type_ = ros_fcp_converter.getBehaviorType();
      robot.setHeight(ros_fcp_converter.getHeight());
      robot.target_.posture_ = ros_fcp_converter.getPosture();

      robot.initialize();
      robot.setSubgoalType(Nagoya);
      robot.target_.type_ = Behavior::Type::CW;
      robot.target_.direction_={std::cos(-60*PI180),std::sin(-60*PI180),0};
      
      for (Leg& leg : robot.legs_){
        if (leg.isLeadingLeg() && leg.isRequestingNewCP()) {
          //RosFcpConnector::publishDebugLog("NewCP",Global_vars::debug_mode);
          std::array<float, 3> cp;
          cp = Comm::preDefinedContactPoint(leg, robot.getHeight(), 0.0f,
                                            robot.bd_.getBehavior());
          robot.setNewCP(leg, cp);
        }
      }
    }
    
    if(ros_fcp_converter.sit_flag_==true){
      Preparation::Stand(robot, motor, 0, Preparation::standing_position,200.0f,2.0f);
      while(1);
    }
  }

  if(ros_fcp_converter.receive_.mode_flag==Behavior::Type::F){
      if(robot.target_.type_ != Behavior::Type::F){
        AdjustingLegs(robot,motor);
        robot.initialize();
        RosFcpConnector::publishFrontLegNum(); 
        //setRecievedData(robot);
      }
      robot.setSubgoalType(Shinmei);
      robot.automaton_.setProcessingType(Shinmei);
      robot.target_.direction_={1.0,0,0};
  }
  if(ros_fcp_converter.receive_.mode_flag==Behavior::Type::CW or ros_fcp_converter.receive_.mode_flag==Behavior::Type::CCW){
    if(robot.target_.type_ != ros_fcp_converter.receive_.mode_flag){
      AdjustingLegs(robot,motor);
      robot.initialize();
    }
    robot.setSubgoalType(Shinmei);
    robot.automaton_.setProcessingType(Nagoya);//robot.automaton_.setProcessingType(Shinmei);
    switch(ros_fcp_converter.receive_.mode_flag){
      case Behavior::Type::CW : robot.target_.direction_={std::cos(-60*PI180),std::sin(-60*PI180),0}; break;
      case Behavior::Type::CCW : robot.target_.direction_={std::cos(60*PI180),std::sin(60*PI180),0}; break;
    }
  }
  robot.target_.type_ = ros_fcp_converter.receive_.mode_flag;

  // if (Global_vars::communication == ROS)
  //   //RosFcpConnector::publishDebugLog("EndsetCP", Global_vars::debug_mode);

  // Serial.println("Read Motor");
  //if (Global_vars::communication == ROS)
    //RosFcpConnector::publishDebugLog("ReadMotor", Global_vars::debug_mode);
  if (!Sensors::readMotor(motor, robot.legs_)) {
    // Error occured
    //robot.stop();
    Termination::termination(ERROR_MOTOR_READ);
  }
  // // Serial.println("Read IMU");
  // if (Global_vars::communication == ROS)
  //   //RosFcpConnector::publishDebugLog("ReadIMU", Global_vars::debug_mode);
  if (!Sensors::readIMU(imu, robot)) {
    // // Serial.println("The posture of the body is unsafe");
    //robot.stop();
    Termination::termination(ERROR_UNSTABLE_POSTURE);
  }

  // //RosFcpConnector::publishDebugLog("Robot Move and write Motor",
  //                                  Global_vars::debug_mode);
  for (Leg& leg: robot.legs_)
    RMPflow2::setPoliciesParam(robot.automaton_.rmpflow_[leg.getID()], ((leg.mode_==2 or leg.mode_==3)?2:1) );
  // // Serial.println("Robot Move and write Motor");
  bool is_fine = robot.Move();
  if (!is_fine) {
    // // Serial.println("Error occured in robot.Move()");
    // if (Global_vars::communication == ROS)
    // //RosFcpConnector::publishDebugLog("Error occured in robot.Move()",
    //                                  Global_vars::debug_mode);
    //robot.stop();
    Termination::termination(ERROR_ROBOT_MOVE);
  }
  if (!Sensors::writeMotor(motor, robot.legs_)) {
    // Error occured
    // if (Global_vars::communication == ROS)
    // //RosFcpConnector::publishDebugLog("Error occured in motor write",
    //                                  Global_vars::debug_mode);
    //robot.stop();
    Termination::termination(ERROR_MOTOR_WRITE);
  }
  // // Serial.println("Finish Movement");
  // if (Global_vars::communication == ROS)
  // //RosFcpConnector::publishDebugLog("Finish Movement",
  //                                  Global_vars::debug_mode);

}

void setup() {
  uint32_t baudrate_serial = 115200;
   Serial.begin(baudrate_serial);
   //Serial.print("11111111111111111111111111111111111");
  
  Config::setupPins();
  {
    robot.initialize();
    Config::setupLegs(robot);
    Config::setupBody(robot);
    Config::setupAutomaton(robot);
    //Config::setupRMPflow(robot);
    Config::setupRMPflow_type(robot);
    Config::setupBehavior(robot);
    
    Sensors::setupIMU(imu);
    Sensors::setupMotor(motor, Motor::OperatingMode::VelocityControl);

    Config::setupForceControl(robot);

    robot.setPeriod(0.013/*CONTROL_PERIOD * 1e-6f*/);
  }
  delay(500);

  // for (size_t i = 0; i < 6; i++) {
  //   for (size_t j = 0; j < 3; j++) {
  //     motor.setPosition(0, i, j);
  //   }
  // }

  // while (true) {
  //   /* code */
  // }

  //Preparation::Raise_under_raise_order(robot, motor);
  //motor.getAllHomingOffset();
  //while(1){};
  /*while(1){
    std::array<float,3> angle{{0.0f,M_PI/4.0f,0.0f}};
    for(Leg& leg : robot.legs_){
      leg.setAngle(angle);
    }
    motor.setSyncAllPosition(robot.legs_);
  }*/

  Preparation::Stand(robot, motor, 0, Preparation::standing_position,200.0f,2.0f);

  /*Global_vars::communication =ROS; // enable for using rosserial
  Global_vars::use_modification_with_estimated_pose =
      false; // enable for using CP modification based on estimated pose*/
  if (Global_vars::communication == ROS)
    RosFcpConnector::setupRos(baudrate_serial);
  Preparation::Stand(robot, motor, 1, Preparation::standing_position,200.0f,5.0f);
  //robot.PostureControlMode();
  //for (Leg &leg : robot.legs_){leg.mode_=4;}
  Timer3.initialize(CONTROL_PERIOD);
  Timer3.attachInterrupt(Movement);

  // Serial.println("Start walking");
}


void loop() {
  // put your main code here, to run repeatedly:
}


void AdjustingLegs(Robot& r, Motor& m){
  //reset_flont_leg_num();
  
  PTP ptp;
  const int speed = 10.0;
  std::array<bool,3> adjusted; adjusted.fill(false);
  
  /*Count swinging legs*/
  int swing_num=0;
  for (Leg& leg: r.legs_){
    if(leg.mode_!=4 and leg.loco_)
      swing_num++;
  }  
  
  if(swing_num!=0){
    if(r.legs_[0].mode_==4 and r.legs_[2].mode_==4 and r.legs_[4].mode_==4){
      swing_num=3;
      r.legs_[1].mode_=1; 
      r.legs_[3].mode_=1;
      r.legs_[5].mode_=1;
    }else if(r.legs_[1].mode_==4 and r.legs_[3].mode_==4 and r.legs_[5].mode_==4){
      swing_num=3;
      r.legs_[0].mode_=1;
      r.legs_[2].mode_=1;
      r.legs_[4].mode_=1;
    }
  }

  if(swing_num!=0){
	  uint8_t ready_legs = 0;
    while (ready_legs<6){
      ready_legs = 0;
      //Ros_pubsub();
      Sensors::readMotor(m, r.legs_);
      for (Leg& leg: r.legs_){
        if(leg.mode_==4){
          ready_legs++;
        }else{
          const int id = leg.getID();
          leg.setAngle(leg.getObsAngle());
          Kinematics::FwdKinematics(leg.getAngle(), leg.getTipLpos(), leg);
          std::array<float, 3> target_pos;
          Kinematics::ConvertBodyToLeg(Preparation::standing_position[id], target_pos, leg);
          target_pos[0] += 40.0;
          target_pos[2] = 0.0;
          if (ptp.point_to_point(leg.getTipLpos(), target_pos, speed) < 5.0) ready_legs++;
          Kinematics::ConvertLegToBody(leg.getTipLpos(), leg.getTipBpos(), leg);
          Kinematics::InvKinematics(leg);
        }
      }
      m.setSyncAllPosition(r.legs_);
    }
    delay(500);

    ready_legs = 0;
    while (ready_legs<6){
      ready_legs = 0;
      //Ros_pubsub();
      Sensors::readMotor(m, r.legs_);
      for (Leg& leg: r.legs_){
        if(leg.mode_==4){
          ready_legs++;
        }else{
          const int id = leg.getID();
          leg.setAngle(leg.getObsAngle());
          Kinematics::FwdKinematics(leg.getAngle(), leg.getTipLpos(), leg);
          std::array<float, 3> target_pos;
          Kinematics::ConvertBodyToLeg(Preparation::standing_position[id], target_pos, leg);
          if (ptp.point_to_point(leg.getTipLpos(), target_pos, speed) < 5.0 or leg.tip_force_[2]<-4.0f) {
            ready_legs++;
            leg.mode_=4;
          }
          Kinematics::ConvertLegToBody(leg.getTipLpos(), leg.getTipBpos(), leg);
          Kinematics::InvKinematics(leg);
        }
      }
      m.setSyncAllPosition(r.legs_);
    }
  }
  delay(1000);
}