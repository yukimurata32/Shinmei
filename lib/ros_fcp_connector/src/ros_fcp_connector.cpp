#include <ros_fcp_connector.h>
#include <main.cpp>

RosFcpConnector::Converter ros_fcp_converter;

namespace RosFcpConnector {

/* node handler */
ros::NodeHandle nh;
/**================
 * Publishers
 ==================*/
/* publishers for previous ground contact points*/
ros::Publisher
    first_previous_point_pub("robot/previous_point",
                             &ros_fcp_converter.first_previous_point_msg_);
ros::Publisher
    second_previous_point_pub("robot/second_previous_point",
                              &ros_fcp_converter.second_previous_point_msg_);
ros::Publisher
    third_previous_point_pub("robot/third_previous_point",
                             &ros_fcp_converter.third_previous_point_msg_);

/* publishers for pose */
ros::Publisher pose_pub("robot/pose", &ros_fcp_converter.pose_msg_);
ros::Publisher imu_pub("robot/imu", &ros_fcp_converter.imu_msg_);

/* publishers for contact point planning */
ros::Publisher
    front_leg_number_pub("robot/front_leg_number",
                         &ros_fcp_converter.planning_leg_number_msg_);
ros::Publisher yaw_init_flag_pub("robot/yaw_init_flag",
                                 &ros_fcp_converter.yaw_init_flag_msg_);
ros::Publisher grounded_flag_pub("robot/front_leg_mode4_flag",
                                 &ros_fcp_converter.grounded_flag_msg_);
ros::Publisher leg_ids_pub("robot/leg_ids_of_previous_points",
                           &ros_fcp_converter.leg_ids_msg_);
/*Motor angle pub*/
ros::Publisher motor_angle_pub("robot/motor_angle_array",
                               &ros_fcp_converter.motor_angles_);
/*Motor evaluation pub*/
ros::Publisher motor_heat_pub("robot/heat_array",
                              &ros_fcp_converter.motor_angles_);
ros::Publisher motor_electric_current_pub("robot/electric_current_array",
                                          &ros_fcp_converter.motor_angles_);
ros::Publisher motor_torque_pub("robot/torque_array",
                                &ros_fcp_converter.motor_angles_);

ros::Publisher log_pub("robot/debug_log", &ros_fcp_converter.debug_log_);

ros::Publisher standby_point_pub("robot/standby_point",&ros_fcp_converter.default_standby_point_);

ros::Publisher leg_mode_pub("robot/leg_mode",&ros_fcp_converter.leg_mode_msg_);

ros::Publisher subgoal_pub("robot/subgoal",&ros_fcp_converter.subgoal_msg_);

ros::Publisher new_cp_pub("robot/new_cp",&ros_fcp_converter.new_cp_msg_);


/**================
 * Subscribers
 ==================*/
ros::Subscriber<std_msgs::Float32MultiArray>
    operated_pos_sub("operate_pos_angle/Serial_out_Pos",
                     &operated_pos_callback);
ros::Subscriber<geometry_msgs::PointStamped>
    planned_point_sub("/planned_contact_point",
                      &planned_point_callback); // test
ros::Subscriber<std_msgs::Bool>
    turn_left_flag_sub("/operate_pos_angle/turn_left_flag",
                       &turn_left_flag_callback);
ros::Subscriber<std_msgs::Bool>
    turn_right_flag_sub("/operate_pos_angle/turn_right_flag",
                        &turn_right_flag_callback);
ros::Subscriber<geometry_msgs::PoseStamped>
    estimated_pose_sub("zed_nodelet/pose", &estimated_pose_callback);
ros::Subscriber<std_msgs::Bool> 
    sit_flag_sub("/operate_pos_angle/sit_flag",&sit_flag_callback);
ros::Subscriber<std_msgs::Float32MultiArray>
    standby_point_sub("/swing_leg_planning/goal_point",&standby_point_callback);
ros::Subscriber<std_msgs::Bool>
    default_standby_point_sub_flag_sub("/swing_leg_planning/default_standby_point_sub_flag",&default_stanby_point_sub_flag_callback);

void setupRos(const long buadrate) {
  nh.getHardware()->setBaud(buadrate);
  nh.initNode();
  nh.advertise(first_previous_point_pub);
  nh.advertise(second_previous_point_pub);
  nh.advertise(third_previous_point_pub);
  nh.advertise(imu_pub);
  nh.advertise(pose_pub);
  nh.advertise(front_leg_number_pub);
  nh.advertise(yaw_init_flag_pub);
  nh.advertise(grounded_flag_pub);
  nh.advertise(leg_ids_pub);
  nh.advertise(motor_angle_pub);
  nh.advertise(log_pub);
  nh.advertise(standby_point_pub);
  nh.advertise(leg_mode_pub);
  nh.advertise(subgoal_pub);
  nh.advertise(new_cp_pub);

  if (parameters.publish_motor_evaluation_data) {
    nh.advertise(motor_electric_current_pub);
    nh.advertise(motor_heat_pub);
    nh.advertise(motor_torque_pub);
  }

  nh.subscribe(planned_point_sub);
  nh.subscribe(operated_pos_sub);
  nh.subscribe(turn_left_flag_sub);
  nh.subscribe(turn_right_flag_sub);
  nh.subscribe(sit_flag_sub);
  nh.subscribe(estimated_pose_sub);
  nh.subscribe(standby_point_sub);
  nh.subscribe(default_standby_point_sub_flag_sub);

  /* initialize values */
  ros_fcp_converter.planning_leg_number_msg_.data = 1;
  if (ros_fcp_converter.planning_leg_number_msg_.data == 1)
    ros_fcp_converter.front_leg_number_ = 4;
  else if (ros_fcp_converter.planning_leg_number_msg_.data == 4)
    ros_fcp_converter.front_leg_number_ = 1;

  ros_fcp_converter.grounded_flag_msg_.data = true;
  ros_fcp_converter.yaw_init_flag_msg_.data = false;

  ros_fcp_converter.receive_.point.x = 0;
  ros_fcp_converter.receive_.point.y = 0;
  ros_fcp_converter.receive_.point.z = 0;

  ros_fcp_converter.receive_.posture.position.x = 0;
  ros_fcp_converter.receive_.posture.position.y = 0;
  ros_fcp_converter.receive_.posture.position.z = 0;
  ros_fcp_converter.receive_.posture.orientation.x = 0;
  ros_fcp_converter.receive_.posture.orientation.y = 0;
  ros_fcp_converter.receive_.posture.orientation.z = 0;
  ros_fcp_converter.receive_.posture.orientation.w = 1;

  ros_fcp_converter.receive_.pose.position.x = 0;
  ros_fcp_converter.receive_.pose.position.y = 0;
  ros_fcp_converter.receive_.pose.position.z = 0;
  ros_fcp_converter.receive_.pose.orientation.x = 0;
  ros_fcp_converter.receive_.pose.orientation.y = 0;
  ros_fcp_converter.receive_.pose.orientation.z = 0;
  ros_fcp_converter.receive_.pose.orientation.w = 1;

  ros_fcp_converter.receive_.pose_diff.position.x = 0;
  ros_fcp_converter.receive_.pose_diff.position.y = 0;
  ros_fcp_converter.receive_.pose_diff.position.z = 0;
  ros_fcp_converter.receive_.pose_diff.orientation.x = 0;
  ros_fcp_converter.receive_.pose_diff.orientation.y = 0;
  ros_fcp_converter.receive_.pose_diff.orientation.z = 0;
  ros_fcp_converter.receive_.pose_diff.orientation.w = 1;

  ros_fcp_converter.receive_.height.data = 300;

  ros_fcp_converter.pose_msg_.pose.position.x = 0.0f;
  ros_fcp_converter.pose_msg_.pose.position.y = 0.0f;
  ros_fcp_converter.pose_msg_.pose.position.z = 0.0f;
  ros_fcp_converter.pose_msg_.pose.orientation.x = 0.0f;
  ros_fcp_converter.pose_msg_.pose.orientation.y = 0.0f;
  ros_fcp_converter.pose_msg_.pose.orientation.z = 0.0f;
  ros_fcp_converter.pose_msg_.pose.orientation.w = 1.0f;

  ros_fcp_converter.new_cp_msg_.point.x=0;
  ros_fcp_converter.new_cp_msg_.point.y=0;
  ros_fcp_converter.new_cp_msg_.point.z=0;

  ros_fcp_converter.leg_ids_msg_.data_length = 4;
  ros_fcp_converter.leg_ids_msg_.data = (int16_t *)malloc(sizeof(int16_t) * 4);
  for (int i = 0; i < 4; ++i) ros_fcp_converter.leg_ids_msg_.data[i] = 0;

  ros_fcp_converter.leg_mode_msg_.data_length = 6;
  ros_fcp_converter.leg_mode_msg_.data = (int16_t *)malloc(sizeof(int16_t) * 6);
  for (int i = 0; i < 6; ++i) ros_fcp_converter.leg_mode_msg_.data[i] = 0;

  ros_fcp_converter.subgoal_msg_.data_length = 3;
  ros_fcp_converter.subgoal_msg_.data = (float *)malloc(sizeof(float) * 3);
  for (int i = 0; i < 3; ++i) ros_fcp_converter.subgoal_msg_.data[i] = 0;

  ros_fcp_converter.default_stanby_point_pub_flag_=false;
  ros_fcp_converter.new_stanby_point_flag_=false;

  ros_fcp_converter.front_leg_num_pub_count_=0;
  ros_fcp_converter.front_leg_num_pub_count_reset_=0;
  ros_fcp_converter.front_leg_num_pub_count_reset_flag_=false;

  ros_fcp_converter.publish_count=0;

  setupMotorArrays();
  setup_standby_point();
  while (nh.spinOnce() != -1)
    ;
}

void default_stanby_point_sub_flag_callback(const std_msgs::Bool &msg){
  ros_fcp_converter.default_stanby_point_pub_flag_=true;
}

void standby_point_callback(const std_msgs::Float32MultiArray &msg){
  /*for(int i=0;i<6;i++){
    for(int j=0;j<3;j++){
      ros_fcp_converter.new_standby_point_[i][j]=float(msg.data[i*3+j]);
    }
  }
  ros_fcp_converter.new_stanby_point_flag_=true;*/
}

void Converter::update_standby_position(Robot &robot) {
  for (Leg &leg : robot.legs_) {
    leg.angle_watcher_.setTimer(millis);
    Kinematics::ConvertBodyToLeg(ros_fcp_converter.new_standby_point_[leg.getID()], leg.standby_point_,leg);
  }
}

void Converter::publish_default_standby_point(std::array<std::array<float, 3>, 6> &msg){

  int count=0;
  for(int i=0;i<6;i++){
    for(int j=0;j<3;j++){
      ros_fcp_converter.default_standby_point_.data[count]=msg[i][j];
      count++;
    }
  }
   if (nh.connected()){
    standby_point_pub.publish(&ros_fcp_converter.default_standby_point_);
   }
  
}

void setup_standby_point(){
  const int number_of_leg = 6;
  const int number_of_xyz = 3;
  ros_fcp_converter.default_standby_point_.layout.dim =
      (std_msgs::MultiArrayDimension *)malloc(
          sizeof(std_msgs::MultiArrayDimension) * 2);
  ros_fcp_converter.default_standby_point_.layout.dim[0].label = "leg_number";
  ros_fcp_converter.default_standby_point_.layout.dim[1].label = "xyz";
  ros_fcp_converter.default_standby_point_.layout.dim[0].size = number_of_leg;
  ros_fcp_converter.default_standby_point_.layout.dim[1].size = number_of_xyz;
  ros_fcp_converter.default_standby_point_.layout.dim[0].stride = number_of_leg * number_of_xyz;
  ros_fcp_converter.default_standby_point_.layout.dim[1].stride = number_of_xyz;
  ros_fcp_converter.default_standby_point_.layout.data_offset = 0;
  ros_fcp_converter.default_standby_point_.data =
      (float *)malloc(sizeof(float) * number_of_leg * number_of_xyz);
  ros_fcp_converter.default_standby_point_.data_length = number_of_leg * number_of_xyz;
}

void setupMotorArrays() {
  // fill out message:
  const int number_of_leg = 6;
  const int number_of_motor = 3;
  ros_fcp_converter.motor_angles_.layout.dim =
      (std_msgs::MultiArrayDimension *)malloc(
          sizeof(std_msgs::MultiArrayDimension) * 2);
  ros_fcp_converter.motor_angles_.layout.dim[0].label = "leg_number";
  ros_fcp_converter.motor_angles_.layout.dim[1].label = "motor_number";
  ros_fcp_converter.motor_angles_.layout.dim[0].size = number_of_leg;
  ros_fcp_converter.motor_angles_.layout.dim[1].size = number_of_motor;
  ros_fcp_converter.motor_angles_.layout.dim[0].stride =
      number_of_leg * number_of_motor;
  ros_fcp_converter.motor_angles_.layout.dim[1].stride = number_of_motor;
  ros_fcp_converter.motor_angles_.layout.data_offset = 0;
  ros_fcp_converter.motor_angles_.data =
      (float *)malloc(sizeof(float) * number_of_leg * number_of_motor);
  ros_fcp_converter.motor_angles_.data_length = number_of_leg * number_of_motor;

  if (parameters.publish_motor_evaluation_data) {
    ros_fcp_converter.motor_heats_.layout.dim =
        (std_msgs::MultiArrayDimension *)malloc(
            sizeof(std_msgs::MultiArrayDimension) * 2);
    ros_fcp_converter.motor_heats_.layout.dim[0].label = "leg_number";
    ros_fcp_converter.motor_heats_.layout.dim[1].label = "motor_number";
    ros_fcp_converter.motor_heats_.layout.dim[0].size = number_of_leg;
    ros_fcp_converter.motor_heats_.layout.dim[1].size = number_of_motor;
    ros_fcp_converter.motor_heats_.layout.dim[0].stride =
        number_of_leg * number_of_motor;
    ros_fcp_converter.motor_heats_.layout.dim[1].stride = number_of_motor;
    ros_fcp_converter.motor_heats_.layout.data_offset = 0;
    ros_fcp_converter.motor_heats_.data =
        (float *)malloc(sizeof(float) * number_of_leg * number_of_motor);
    ros_fcp_converter.motor_heats_.data_length =
        number_of_leg * number_of_motor;

    ros_fcp_converter.motor_torques_.layout.dim =
        (std_msgs::MultiArrayDimension *)malloc(
            sizeof(std_msgs::MultiArrayDimension) * 2);
    ros_fcp_converter.motor_torques_.layout.dim[0].label = "leg_number";
    ros_fcp_converter.motor_torques_.layout.dim[1].label = "motor_number";
    ros_fcp_converter.motor_torques_.layout.dim[0].size = number_of_leg;
    ros_fcp_converter.motor_torques_.layout.dim[1].size = number_of_motor;
    ros_fcp_converter.motor_torques_.layout.dim[0].stride =
        number_of_leg * number_of_motor;
    ros_fcp_converter.motor_torques_.layout.dim[1].stride = number_of_motor;
    ros_fcp_converter.motor_torques_.layout.data_offset = 0;
    ros_fcp_converter.motor_torques_.data =
        (float *)malloc(sizeof(float) * number_of_leg * number_of_motor);
    ros_fcp_converter.motor_torques_.data_length =
        number_of_leg * number_of_motor;

    ros_fcp_converter.motor_electric_currents_.layout.dim =
        (std_msgs::MultiArrayDimension *)malloc(
            sizeof(std_msgs::MultiArrayDimension) * 2);
    ros_fcp_converter.motor_electric_currents_.layout.dim[0].label =
        "leg_number";
    ros_fcp_converter.motor_electric_currents_.layout.dim[1].label =
        "motor_number";
    ros_fcp_converter.motor_electric_currents_.layout.dim[0].size =
        number_of_leg;
    ros_fcp_converter.motor_electric_currents_.layout.dim[1].size =
        number_of_motor;
    ros_fcp_converter.motor_electric_currents_.layout.dim[0].stride =
        number_of_leg * number_of_motor;
    ros_fcp_converter.motor_electric_currents_.layout.dim[1].stride =
        number_of_motor;
    ros_fcp_converter.motor_electric_currents_.layout.data_offset = 0;
    ros_fcp_converter.motor_electric_currents_.data =
        (float *)malloc(sizeof(float) * number_of_leg * number_of_motor);
    ros_fcp_converter.motor_electric_currents_.data_length =
        number_of_leg * number_of_motor;
  }
}

void publishData() {
  if (nh.connected()) {
    first_previous_point_pub.publish(
        &ros_fcp_converter.first_previous_point_msg_);

    second_previous_point_pub.publish(
        &ros_fcp_converter.second_previous_point_msg_);

    third_previous_point_pub.publish(
        &ros_fcp_converter.third_previous_point_msg_);

    /*leg_ids_pub.publish(&ros_fcp_converter.leg_ids_msg_);*/

    imu_pub.publish(&ros_fcp_converter.imu_msg_);

    pose_pub.publish(&ros_fcp_converter.pose_msg_);    

    /*motor_angle_pub.publish(&ros_fcp_converter.motor_angles_);
    if (parameters.publish_motor_evaluation_data) {
      motor_heat_pub.publish(&ros_fcp_converter.motor_heats_);
      motor_torque_pub.publish(&ros_fcp_converter.motor_torques_);
      // motor_electric_current_pub.publish(
      //     &ros_fcp_converter.motor_electric_currents_);
    }*/
  }
}

void publishFrontLegNum() {
  front_leg_number_pub.publish(&ros_fcp_converter.planning_leg_number_msg_);

  /*if(ros_fcp_converter.front_leg_num_pub_count_reset_flag_==false){
    if(ros_fcp_converter.front_leg_num_pub_count_==0){
      front_leg_number_pub.publish(&ros_fcp_converter.planning_leg_number_msg_);
    }
    ros_fcp_converter.front_leg_num_pub_count_++;
    if(ros_fcp_converter.front_leg_num_pub_count_>=100){
      ros_fcp_converter.front_leg_num_pub_count_=0;
    }
  }else{
    ros_fcp_converter.front_leg_num_pub_count_reset_++;
    if(ros_fcp_converter.front_leg_num_pub_count_reset_>=1){
      ros_fcp_converter.front_leg_num_pub_count_reset_flag_=false;
    }
  }*/




  /*if(ros_fcp_converter.front_leg_num_pub_count_reset_flag_==true){
    ros_fcp_converter.front_leg_num_pub_count_=0;
    ros_fcp_converter.front_leg_num_pub_count_reset_flag_=false;
  }

  if(nh.connected()==false and ros_fcp_converter.front_leg_num_pub_count_==0) ros_fcp_converter.front_leg_num_pub_count_reset_flag_=true;

  if (nh.connected() and ros_fcp_converter.front_leg_num_pub_count_==0 and ros_fcp_converter.front_leg_num_pub_count_reset_flag_==false){
    front_leg_number_pub.publish(&ros_fcp_converter.planning_leg_number_msg_);
  }
  if(nh.connected() and ros_fcp_converter.front_leg_num_pub_count_<80 and ros_fcp_converter.front_leg_num_pub_count_reset_flag_==false) ros_fcp_converter.front_leg_num_pub_count_++;
  else if(ros_fcp_converter.front_leg_num_pub_count_>=80) ros_fcp_converter.front_leg_num_pub_count_=0;*/

  
}

void publishNewCP(std::array<float, 3> cp){
  ros_fcp_converter.new_cp_msg_.point.x=cp[0]/1000;
  ros_fcp_converter.new_cp_msg_.point.y=cp[1]/1000;
  ros_fcp_converter.new_cp_msg_.point.z=cp[2]/1000;

  new_cp_pub.publish(&ros_fcp_converter.new_cp_msg_);
}

void publishYawInitFlag() {
  if (nh.connected()) {
    ros_fcp_converter.yaw_init_flag_msg_.data = true;
    yaw_init_flag_pub.publish(&ros_fcp_converter.yaw_init_flag_msg_);
    ros_fcp_converter.yaw_init_flag_msg_.data = false;
  }
}

void publishDebugLog(const char *log_msg, const bool debug_mode) {
  if (nh.connected() and debug_mode) {
    ros_fcp_converter.debug_log_.data = log_msg;
    log_pub.publish(&ros_fcp_converter.debug_log_);
  }
}

void publishLegMode(Robot &robot){
  for(int i=0;i<6;i++){
   ros_fcp_converter.leg_mode_msg_.data[i]=robot.legs_[i].mode_;
   //ros_fcp_converter.leg_mode_msg_.data[i]=robot.legs_[i].getOtherSideLeg().point_;
  }
  leg_mode_pub.publish(&ros_fcp_converter.leg_mode_msg_);
}

void publishSubGoal(Robot &robot){
  for(int i=0;i<3;i++){
    ros_fcp_converter.subgoal_msg_.data[i]=robot.subgoal_[i]/1000;
  }
  subgoal_pub.publish(&ros_fcp_converter.subgoal_msg_);
}

void spin(Robot &robot) {
  nh.spinOnce();
  publishSubGoal(robot);
  if(ros_fcp_converter.publish_count==0){
     publishData();
     publishLegMode(robot);
     //publishSubGoal(robot);
     ros_fcp_converter.publish_count+=1;
  }
  else if(ros_fcp_converter.publish_count>=5){
    ros_fcp_converter.publish_count=0;
  }
  else {
    ros_fcp_converter.publish_count+=1;
  }
}

/**====================
 * Callback functions
 ======================*/
void operated_pos_callback(const std_msgs::Float32MultiArray &pos_msg) {
  ros_fcp_converter.receive_.posture.position.x = pos_msg.data[0] * PI180;
  ros_fcp_converter.receive_.posture.position.y = pos_msg.data[1] * PI180;
  ros_fcp_converter.receive_.posture.position.z = pos_msg.data[2] * PI180;
  ros_fcp_converter.receive_.height.data = pos_msg.data[3];
  return;
}

void planned_point_callback(const geometry_msgs::PointStamped &point_msg) {
  ros_fcp_converter.receive_.point.x = point_msg.point.x * 1000;
  ros_fcp_converter.receive_.point.y = point_msg.point.y * 1000;
  ros_fcp_converter.receive_.point.z = point_msg.point.z * 1000;

  ros_fcp_converter.front_leg_num_pub_count_reset_=0;
  ros_fcp_converter.front_leg_num_pub_count_reset_flag_=true;
  ros_fcp_converter.front_leg_num_pub_count_=0;
  //publishDebugLog("planned", true);
  return;
}

void estimated_pose_callback(const geometry_msgs::PoseStamped &pose_msg) {
  ros_fcp_converter.receive_.pose_diff.position.x =
      ros_fcp_converter.receive_.pose.position.x - pose_msg.pose.position.x;
  ros_fcp_converter.receive_.pose_diff.position.y =
      ros_fcp_converter.receive_.pose.position.y - pose_msg.pose.position.y;
  ros_fcp_converter.receive_.pose_diff.position.z =
      ros_fcp_converter.receive_.pose.position.z - pose_msg.pose.position.z;

  /// @brief quaternion diff copydetails
  ros_fcp_converter.receive_.pose_diff.orientation.x =
      ros_fcp_converter.receive_.pose.orientation.w *
          pose_msg.pose.orientation.x -
      ros_fcp_converter.receive_.pose.orientation.x *
          pose_msg.pose.orientation.w -
      ros_fcp_converter.receive_.pose.orientation.y *
          pose_msg.pose.orientation.z +
      ros_fcp_converter.receive_.pose.orientation.z *
          pose_msg.pose.orientation.y;
  ros_fcp_converter.receive_.pose_diff.orientation.y =
      ros_fcp_converter.receive_.pose.orientation.w *
          pose_msg.pose.orientation.y -
      ros_fcp_converter.receive_.pose.orientation.y *
          pose_msg.pose.orientation.w -
      ros_fcp_converter.receive_.pose.orientation.z *
          pose_msg.pose.orientation.x +
      ros_fcp_converter.receive_.pose.orientation.x *
          pose_msg.pose.orientation.z;
  ros_fcp_converter.receive_.pose_diff.orientation.z =
      ros_fcp_converter.receive_.pose.orientation.w *
          pose_msg.pose.orientation.z -
      ros_fcp_converter.receive_.pose.orientation.z *
          pose_msg.pose.orientation.w -
      ros_fcp_converter.receive_.pose.orientation.x *
          pose_msg.pose.orientation.y +
      ros_fcp_converter.receive_.pose.orientation.y *
          pose_msg.pose.orientation.x;
  ros_fcp_converter.receive_.pose_diff.orientation.w =
      ros_fcp_converter.receive_.pose.orientation.w *
          pose_msg.pose.orientation.w +
      ros_fcp_converter.receive_.pose.orientation.x *
          pose_msg.pose.orientation.x +
      ros_fcp_converter.receive_.pose.orientation.y *
          pose_msg.pose.orientation.y +
      ros_fcp_converter.receive_.pose.orientation.z *
          pose_msg.pose.orientation.z;
  //////////

  ros_fcp_converter.receive_.pose = pose_msg.pose;
} // namespace RosFcpConnector

void turn_left_flag_callback(const std_msgs::Bool &turn_flag) {
  /*if (turn_flag.data)
    ros_fcp_converter.receive_.mode_flag = Behavior::Type::CCW;
  else
    ros_fcp_converter.receive_.mode_flag = Behavior::Type::F;*/
}

void turn_right_flag_callback(const std_msgs::Bool &turn_flag) {
  /*if (turn_flag.data){
    ros_fcp_converter.receive_.mode_flag = Behavior::Type::CW;
    ros_fcp_converter.receive_.point.x = 320;
    ros_fcp_converter.receive_.point.y = -200;
    ros_fcp_converter.receive_.point.z = -200;}
  else
    ros_fcp_converter.receive_.mode_flag = Behavior::Type::F;*/
}

void sit_flag_callback(const std_msgs::Bool &sit_flag){
  ros_fcp_converter.sit_flag_ = sit_flag.data;

}

void Converter::setPlanningLegNumber(const Robot &robot){
  for (const Leg &leg : robot.legs_) {
    /* leading leg for the next contact point is planned */
    if (leg.isRequestingNewCP() && leg.getID() == leg.getFollowID()) {
      planning_leg_number_msg_.data = leg.getID();
    }
  }
}

void Converter::setThreeLatestContactPoints(const Robot &robot) {
  /* choose a leg for which the next contact point is planned */
  /* set positions of the three latest points */
  int latest_point = robot.getLatestPoint();
  short ids[4] = {-1, -1, -1, -1}; // -1 means there is not a corresponding leg.
  for (const Leg &leg : robot.legs_) {
    /* leading leg for the next contact point is planned */
    if (leg.isRequestingNewCP() && leg.getID() == leg.getFollowID()) {
      ids[0] = leg.getID();
      planning_leg_number_msg_.data = leg.getID();
    }
    /* the three latest contact point*/
    else if (leg.mode_ == 4 && latest_point - leg.point_ < 3) {
      // if the leg shares the contact point with rear leg, the contact position
      // of the rear leg is selected.
      if (leg.getID() != leg.getFollowedID() &&
          leg.point_ == leg.getFollowedLeg().point_)
        continue;
      int order = latest_point - leg.point_ + 1;
      ids[order] = leg.getID();
      /*getPreviousPoint(order).point.x = leg.node_bpos_[3][0] / 1000;
      getPreviousPoint(order).point.y = leg.node_bpos_[3][1] / 1000;
      getPreviousPoint(order).point.z = leg.node_bpos_[3][2] / 1000;*/
      getPreviousPoint(order).point.x = leg.observed_node_bpos_[3][0] / 1000;
      getPreviousPoint(order).point.y = leg.observed_node_bpos_[3][1] / 1000;
      getPreviousPoint(order).point.z = leg.observed_node_bpos_[3][2] / 1000;
    }
  }
  for (int i = 0; i < leg_ids_msg_.data_length; ++i)
    leg_ids_msg_.data[i] = ids[i];

  auto now_ros_time = nh.now();
  for (int i = 1; i < 4; ++i) {
    getPreviousPoint(i).header.stamp = now_ros_time;
    getPreviousPoint(i).header.frame_id = frame_name;
  }
}

void Converter::setPose(const Robot &robot) {
  EulerAnglesToQuaternion(robot.body_.getPosture(), pose_msg_.pose.orientation);
  pose_msg_.pose.position.x += robot.body_.mileage_[0] / 1000;
  pose_msg_.pose.position.y += robot.body_.mileage_[1] / 1000;
  // pose_msg_.pose.position.x = 0;
  // pose_msg_.pose.position.y = 0;
  pose_msg_.pose.position.z = robot.getHeight() / 1000;

  pose_msg_.header.stamp = nh.now();
}

void Converter::setImu(const Robot &robot) {
  imu_msg_.linear_acceleration.x = robot.body_.accel_[0];
  imu_msg_.linear_acceleration.y = robot.body_.accel_[1];
  imu_msg_.linear_acceleration.z = robot.body_.accel_[2];

  imu_msg_.angular_velocity.x = 0.0f; // maybe robot.body_.getPostureVel();
  imu_msg_.angular_velocity.y = 0.0f;
  imu_msg_.angular_velocity.z = 0.0f;

  EulerAnglesToQuaternion(robot.getPosture(), imu_msg_.orientation);
  imu_msg_.header.stamp = nh.now();
}

void Converter::setMotorAngle(
    std::array<std::array<float, 3>, 6> &motor_angle_array) {
  int motor_count = 0;
  for (auto &motor_angles : motor_angle_array) {
    for (auto &angle : motor_angles) {
      ros_fcp_converter.motor_angles_.data[motor_count] = angle;
      motor_count++;
    }
  }
}
void Converter::setMotorEvaluationData(
    std::array<std::array<float, 3>, 6> &motor_torque_array,
    std::array<std::array<float, 3>, 6> &motor_heat_array) {
  if (!parameters.publish_motor_evaluation_data) {
    return;
  }
  int motor_count = 0;
  // for (int i = 0; i < 6; i++) {
  //   for (int j = 0; j < 3; j++) {
  //     ros_fcp_converter.motor_torques_.data[motor_count] =
  //         motor_torque_array[i][j];
  //     ros_fcp_converter.motor_heats_.data[motor_count] =
  //         motor_torque_array[i][j];
  //     motor_count++;
  //   }
  // }
  for (auto &motor_torques : motor_torque_array) {
    for (auto &torque : motor_torques) {
      ros_fcp_converter.motor_torques_.data[motor_count] = torque;
      motor_count++;
    }
  }
  motor_count = 0;
  for (auto &motor_heats : motor_torque_array) {
    for (auto &heat : motor_heats) {
      ros_fcp_converter.motor_heats_.data[motor_count] = heat;
      motor_count++;
    }
  }
}

std::array<float, 3> Converter::getContactPoint() {
  std::array<float, 3> cp{
      {receive_.point.x, receive_.point.y, receive_.point.z}};
  return cp;
}

std::array<float, 3> Converter::getPosture() {
  std::array<float, 3> pos{{receive_.posture.position.x,
                            receive_.posture.position.y,
                            receive_.posture.position.z}};

  return pos;
}

float Converter::getHeight() { return receive_.height.data; }

Behavior::Type Converter::getBehaviorType() { return receive_.mode_flag; }

void reset_flont_leg_num(){
  ros_fcp_converter.planning_leg_number_msg_.data=1;
  ros_fcp_converter.front_leg_number_=4;

  /* receive_.point.x=0;
  receive_.point.y=0;
  receive_.point.z=0; */
  
}

} // namespace RosFcpConnector

