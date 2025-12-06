#ifndef CHASSIS_TASK_HPP
#define CHASSIS_TASK_HPP
#include "HERO_SELECTION.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor/super_cap/super_cap.hpp"
#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/mecanum/mecanum.hpp"
#include "tools/pid/pid.hpp"

//根据剩余能量和电容情况确定最大功率
void Pmax_get();
//基于缓冲能量的功率控制
void buff_energy_p_limited();

// -------------------- 控制参数 --------------------
constexpr float T_CHASSIS = 1e-3f;  // 控制周期, 单位: s
//底盘
constexpr float CAP_MAX_POWER = 400.0f;  //16V时电容最大放电功率
//电容最大放电功率=25*电压
constexpr float WHEEL_RADIUS = 77e-3f;       // m
constexpr float CHASSIS_LENGTH = 396.2e-3f;  // m
constexpr float CHASSIS_WIDTH = 356.47e-3f;  // m

inline sp::PID chassis_follow_wz_pid(T_CHASSIS, 8.5f, 0.0f, 0.7f, 5.0f, 3.0f, 0.5f);

inline sp::SuperCap super_cap(sp::SuperCapMode::AUTOMODE);
inline sp::RM_Motor wheel_lf(1, sp::RM_Motors::M3508, RADUCTION_RATIO);  // left front
inline sp::RM_Motor wheel_lr(4, sp::RM_Motors::M3508, RADUCTION_RATIO);  // left rear
inline sp::RM_Motor wheel_rf(2, sp::RM_Motors::M3508, RADUCTION_RATIO);  // right front
inline sp::RM_Motor wheel_rr(3, sp::RM_Motors::M3508, RADUCTION_RATIO);  // right rear

inline sp::LowPassFilter yaw_relative_angle_filter(0.01f);

// inline sp::PID chassis_follow_wz_pid(0.001f, 8.5f, 0.0f, 0.7f, 5.0f, 3.0f, 0.5f);

//小陀螺角速度rad/s
constexpr float SPIN_W = 10.0f;  //大约112w
//遥控器平移最大速度
constexpr float REMOTE_CONTROL_V = 3.0f;

typedef struct
{
  float vx;
  float vy;
  float wz;
} Chassis_Speed;

typedef struct
{
  float lf;
  float lr;
  float rf;
  float rr;
} Wheel_Speed;

typedef struct
{
  float lf;
  float lr;
  float rf;
  float rr;
} Wheel_Torque;

//----对外调试
extern sp::Mecanum chassis;
extern Wheel_Torque wheel_give_torque;
extern Wheel_Speed chassis_target_speed;
extern Wheel_Speed wheel_speed;

#endif