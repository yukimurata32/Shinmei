#pragma once
#include <array>
#include <types.h>

#define COMMAND_STOP 0.1f
#define ERROR_ROBOT_MOVE 0.5f
#define ERROR_MOTOR_WRITE 1.0f
#define ERROR_MOTOR_READ 2.0f
#define ERROR_UNSTABLE_POSTURE 3.0f
#define ERROR_RING_BUFF 4.0f
#define ERROR_SD_CARD_OPENING 5.0f

#define CONTROL_PERIOD 10000 // microseconds

// about robot frame
std::array<std::array<float, 3>, 6> RELATIVE_POS{{{170.0, -12.0, 0.0},
                                                  {0.0, -12.0, 0.0},
                                                  {-170.0, -12.0, 0.0},
                                                  {-170.0, 12.0, 0.0},
                                                  {0.0, 12.0, 0.0},
                                                  {170.0, 12.0, 0.0}}};
std::array<float, 6> RELATIVE_ANGLE{{-M_PI / 3, -M_PI / 2, -M_PI * 2 / 3,
                                     -M_PI * 4 / 3, -M_PI * 3 / 2,
                                     -M_PI * 5 / 3}};
std::array<float, 4> LINK_LENGTH{{35.355, 109.72, 249.861, 255.0}};
std::array<float, 4> LINK_RADIUS{{0.0f, 0.0f, 22.0f, 10.0f}};
float INCLINATION = M_PI / 4;
float BODY_HEIGHT = 200.0f;
//constexpr std::array<float, 3> COG{{0.0, 0.0, 0.0}};
constexpr std::array<float, 3> COG{{0.0, 0.0, 0.0}}; //-21.5

// for RMPflow parameter
const int ATTRACTOR_TYPE = 2;
std::array<float, 6> LENGHT_RMPFLOW{
    {35.355f, 109.72f, 221.121f, 58.996f, 54.75f, LINK_LENGTH[3] - 54.75f}};
std::array<float, 6> RADIUS_RMPFLOW{{0.0f, 0.0f, 40.0f, 40.0f, 40.0f, 10.0f}};
float ANGLE_BETWEEN_CYLINDER12 = 12.584f / 180.0f * M_PI;
float ANGLE_BETWEEN_CYLINDER23 = 54.746f / 180.0f * M_PI;

// Phisical parameter
const std::array<float, 4> CENTER_OF_LINK{
    {17.7, 40.0, 88.0, 42.0}}; //[mm] center of mass
const std::array<float, 4> MASS_OF_LINK{{0.0, 0.170, 0.445, 0.285}}; //[kg]
float gravitational_acc = 9.80665; //[m/sec^2] 重力加速度
std::array<float, 3> gravitational_acc_vector = {0, 0, -gravitational_acc};

/* for body*/
float BODY_GAIN_POSTURE = 0.1f;//0.050f;
float BODY_GAIN_POSTURE_VEL = 0.00f;
float BODY_SATURATION_POSTURE = 5.0f * PI180;
float BODY_POSTURE_DEADZONE = 0.5f * PI180;
float BODY_STABLE_MARGIN = 50.0f;//80.0f;

/*float JARK = 4900*2.25;
float M_ACC = 400*2.25;
float m_VEL = 1200*2.25;*/

float JARK = 4900*2;
float M_ACC = 400*2;
float m_VEL = 1200*2;

// float JARK =70000*0.4*2.0*CONTROL_PERIOD*1e-6;
// float M_ACC =40000*0.4*2.0*CONTROL_PERIOD*1e-6;
// float m_VEL =30000*0.4*2.0*CONTROL_PERIOD*1e-6;
float ITERATION = 20;
const std::array<float, 5> LEG_SPEED{{0.0, 40.0, 0.0, 40.0, 20.0}};

/* for automaton */
float ANGLE_VEL_LIMIT = 2.5f;

/* for motor */
float MOTOR_LIMIT_VEL = 350;
float MOTOR_LIMIT_ACC = 32000;
float DYNAMIXEL2PI_VEL = 0.02398082392; // 0.229*2*pi/60
float DYNAMIXEL2PI_ACC = 0.3745075149;  // 214.577*2*pi/3600
float SYNC_INTERVAL_MILLIS = 500;

namespace Global_vars {
bool orange_led = true;
int one_leg_mode = -1;
CommType communication = ALONE; //ROS; 
bool use_modification_with_estimated_pose = false;
bool use_sd_card = false;
bool debug_mode = false;
bool fast_planning_mode=true;
bool publish_motor_data=false;
} // namespace Global_vars

int INIT_DIRECTION = 0;

/* from arduino */
int last_read_leg_num = 0, wrecv_flag = 0;
std::array<float, 3> global_dir_ = {1, 0, 0};
std::array<float, 3> initial_global_dir_ = {1, 0, 0};
std::array<float, 3> subgoal_log_ = {0,0,0};

/**
 * sd card
 */
#define SD_CONFIG SdioConfig(FIFO_SDIO) // Use Teensy SDIO
#define LOG_INTERVAL_USEC                                                      \
  10000 // Interval between points for 100 sps. (samples per second)
#define LOG_FILE_SIZE                                                          \
  2400 * 180 * 100 // 432,000 bytes.// Size to log 4*600 byte lines at 100 Hz
                   // for more than three minutes.
#define RING_BUF_CAPACITY                                                      \
  512 * 10 // Space to hold more than 800 ms of data for 10 byte lines at 25
           // ksps.

// initial adjacent information for each direction
constexpr std::array<std::array<int, 6>, 6> F_front_list{{{0, 0, 1, 4, 5, 5},
                                                          {0, 1, 1, 2, 5, 0},
                                                          {1, 1, 2, 2, 3, 0},
                                                          {1, 2, 2, 3, 3, 4},
                                                          {5, 2, 3, 3, 4, 4},
                                                          {5, 0, 3, 4, 4, 5}}};

constexpr std::array<std::array<int, 6>, 6> F_rear_list{{{1, 2, 2, 3, 3, 4},
                                                         {5, 2, 3, 3, 4, 4},
                                                         {5, 0, 3, 4, 4, 5},
                                                         {0, 0, 1, 4, 5, 5},
                                                         {0, 1, 1, 2, 5, 0},
                                                         {1, 1, 2, 2, 3, 0}}};

constexpr std::array<std::array<int, 6>, 6> CW_front_list{{{1, 2, 2, 4, 5, 5},
                                                           {0, 2, 3, 3, 5, 0},
                                                           {1, 1, 3, 4, 4, 0},
                                                           {1, 2, 2, 4, 5, 5},
                                                           {0, 2, 3, 3, 5, 0},
                                                           {1, 1, 3, 4, 4, 0}}};

constexpr std::array<std::array<int, 6>, 6> CW_rear_list{{{0, 0, 1, 3, 3, 4},
                                                          {5, 1, 1, 2, 4, 4},
                                                          {5, 0, 2, 2, 3, 5},
                                                          {0, 0, 1, 3, 3, 4},
                                                          {5, 1, 1, 2, 4, 4},
                                                          {5, 0, 2, 2, 3, 5}}};

constexpr std::array<std::array<int, 6>, 6> CCW_front_list = CW_rear_list;
// {{
//     {0, 0, 1, 3, 3, 4},
//     {5, 1, 1, 2, 4, 4},
//     {5, 0, 2, 2, 3, 5},
//     {0, 0, 1, 3, 3, 4},
//     {5, 1, 1, 2, 4, 4},
//     {5, 0, 2, 2, 3, 5}

// }};

constexpr std::array<std::array<int, 6>, 6> CCW_rear_list = CW_front_list;
// {{
//     {1, 2, 2, 4, 5, 5},
//     {0, 2, 3, 3, 5, 0},
//     {1, 1, 3, 4, 4, 0},
//     {1, 2, 2, 4, 5, 5},
//     {0, 2, 3, 3, 5, 0},
//     {1, 1, 3, 4, 4, 0}
// }};

// Added in Shinmei
constexpr std::array<std::array<int, 6>, 6> other_side_list{
    {{5, 4, 3, 2, 1, 0},
     {1, 0, 5, 4, 3, 2},
     {3, 2, 1, 0, 5, 4},
     {5, 4, 3, 2, 1, 0},
     {1, 0, 5, 4, 3, 2},
     {3, 2, 1, 0, 5, 4}}};

//
const int kFrontListCrab_[6][6] = {
    {0, 1, 1, 0, 5, 5}, {0, 1, 2, 2, 1, 0}, {1, 1, 2, 3, 3, 2},
    {3, 2, 2, 3, 4, 4}, {5, 4, 3, 3, 4, 5}, {0, 0, 5, 4, 4, 5},
};
const int kRearListCrab_[6][6] = {{3, 2, 2, 3, 4, 4}, {5, 4, 3, 3, 4, 5},
                                  {0, 0, 5, 4, 4, 5}, {0, 1, 1, 0, 5, 5},
                                  {0, 1, 2, 2, 1, 0}, {1, 1, 2, 3, 3, 2}};
const int kLegColumnCrab_[6][3][2] = {
    {{5, 4}, {0, 3}, {1, 2}}, {{0, 5}, {1, 4}, {2, 3}},
    {{1, 0}, {2, 5}, {3, 4}}, {{2, 1}, {3, 0}, {4, 5}},
    {{3, 2}, {4, 1}, {5, 0}}, {{4, 3}, {5, 2}, {0, 1}},
};
const int kLegColumn_[6][2][3] = {
    {{0, 1, 2}, {5, 4, 3}}, {{1, 2, 3}, {0, 5, 4}}, {{2, 3, 4}, {1, 0, 5}},
    {{3, 4, 5}, {2, 1, 0}}, {{4, 5, 0}, {3, 2, 1}}, {{5, 0, 1}, {4, 3, 2}}};