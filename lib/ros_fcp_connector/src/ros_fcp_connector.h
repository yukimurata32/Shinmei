#include <ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

/* std_msgs */
#include <std_msgs/Bool.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Int16MultiArray.h>
#include <std_msgs/String.h>

/* geometry msgs*/
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/PoseStamped.h>

/* sensor msgs */
#include <sensor_msgs/Imu.h>

#include <cmath>
#include <parameters.h>
#include <robot.h>
#include <string>

namespace RosFcpConnector {

class Converter {
private:
  const char *frame_name = "base_link";

public:
  /**====================
   * Received data
   ======================*/
  struct {
    geometry_msgs::Pose posture;
    geometry_msgs::Point point;
    geometry_msgs::Pose pose;
    geometry_msgs::Pose pose_diff;
    std_msgs::Int16 height;
    Behavior::Type mode_flag;
  } receive_;

  /**====================
   * Messages for Publish
   ======================*/
  /* msgs for previous ground contact points*/
  geometry_msgs::PointStamped first_previous_point_msg_;
  geometry_msgs::PointStamped second_previous_point_msg_;
  geometry_msgs::PointStamped third_previous_point_msg_;

  /* msgs for pose */
  sensor_msgs::Imu imu_msg_;
  geometry_msgs::PoseStamped pose_msg_;

  /* msgs for contact point planning */
  std_msgs::Int16 planning_leg_number_msg_;
  std_msgs::Int16MultiArray leg_ids_msg_;
  std_msgs::Bool yaw_init_flag_msg_;
  std_msgs::Bool grounded_flag_msg_;
  /*Motor angle msg*/
  std_msgs::Float64MultiArray motor_angles_;
  std_msgs::Float64MultiArray motor_heats_;
  std_msgs::Float64MultiArray motor_torques_;
  std_msgs::Float64MultiArray motor_electric_currents_;
  std_msgs::String debug_log_;

  std_msgs::Int16MultiArray leg_mode_msg_;
  std_msgs::Float64MultiArray subgoal_msg_;


  int delay_time_ = 0;
  int publish_count;
  int front_leg_number_;
  bool sit_flag_ = false;
  bool default_stanby_point_pub_flag_;
  int front_leg_num_pub_count_;
  int front_leg_num_pub_count_reset_;
  bool front_leg_num_pub_count_reset_flag_;

  std_msgs::Float64MultiArray default_standby_point_;

  std::array<std::array<float, 3>, 6> new_standby_point_;
  bool new_stanby_point_flag_;

  geometry_msgs::PointStamped new_cp_msg_;

public:
  Converter() {}
  ~Converter() { free(leg_ids_msg_.data);
                 free(leg_mode_msg_.data);
                 free(subgoal_msg_.data);
                 free(default_standby_point_.data);
                 free(motor_angles_.data); 
                 free(motor_heats_.data);
                 free(motor_torques_.data);
                 free(motor_electric_currents_.data);
                 
                 }

  void publish_default_standby_point(std::array<std::array<float, 3>, 6> &msg);
  void setPlanningLegNumber(const Robot &robot);
  void setThreeLatestContactPoints(const Robot &robot);
  void setPose(const Robot &robot);
  void setImu(const Robot &robot);
  void setMotorAngle(std::array<std::array<float, 3>, 6> &motor_angle_array);
  void setMotorEvaluationData(
      std::array<std::array<float, 3>, 6> &motor_torque_array,
      std::array<std::array<float, 3>, 6> &motor_heat_array);

  std::array<float, 3> getContactPoint();
  std::array<float, 3> getPosture();
  float getHeight();
  Behavior::Type getBehaviorType();
  void update_standby_position(Robot &r);
  

private:
  inline geometry_msgs::PointStamped &getPreviousPoint(const int i) {
    switch(i){
      case 1: return first_previous_point_msg_; break;
      case 2: return second_previous_point_msg_; break;
      case 3: return third_previous_point_msg_; break;
      default: return first_previous_point_msg_; break;
    }
    /*if (i == 1)
      return first_previous_point_msg_;
    if (i == 2)
      return second_previous_point_msg_;
    if (i == 3)
      return third_previous_point_msg_;

    return first_previous_point_msg_; // as an exception, need improvement
    */
  }

  inline void EulerAnglesToQuaternion(const std::array<float, 3> &rpy,
                                      geometry_msgs::Quaternion &q) {
    double cosRoll = std::cos(rpy[0] * 0.5f);
    double sinRoll = std::sin(rpy[0] * 0.5f);
    double cosPitch = std::cos(rpy[1] * 0.5f);
    double sinPitch = std::sin(rpy[1] * 0.5f);
    double cosYaw = std::cos(rpy[2] * 0.5f);
    double sinYaw = std::sin(rpy[2] * 0.5f);

    q.w = cosRoll * cosPitch * cosYaw + sinRoll * sinPitch * sinYaw;
    q.x = sinRoll * cosPitch * cosYaw - cosRoll * sinPitch * sinYaw;
    q.y = cosRoll * sinPitch * cosYaw + sinRoll * cosPitch * sinYaw;
    q.z = cosRoll * cosPitch * sinYaw - sinRoll * sinPitch * cosYaw;
  }
};

void setupRos(const long buadrate);
void setupMotorArrays();
void publishData();
void publishFrontLegNum();
void publishYawInitFlag();
void publishDebugLog(const char *log_msg, const bool debug_mode);
void spin(Robot &robot);
void setup_standby_point();
void publishLegMode(Robot &robot);
void publishSubGoal(Robot &robot);
void publishNewCP(std::array<float, 3> cp);

/**====================
 * Callback functions
 ======================*/
void standby_point_callback(const std_msgs::Float32MultiArray &msg);
void operated_pos_callback(const std_msgs::Float32MultiArray &pos_msg);
void planned_point_callback(const geometry_msgs::PointStamped &point_msg);
void estimated_pose_callback(const geometry_msgs::PoseStamped &pose_msg);
void turn_left_flag_callback(const std_msgs::Bool &turn_flag);
void turn_right_flag_callback(const std_msgs::Bool &turn_flag);
void sit_flag_callback(const std_msgs::Bool &sit_flag);
void default_stanby_point_sub_flag_callback(const std_msgs::Bool &msg);
} // namespace RosFcpConnector

extern RosFcpConnector::Converter ros_fcp_converter;