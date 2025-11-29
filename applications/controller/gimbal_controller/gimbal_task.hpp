#ifndef GIMBAL_TASK_HPP
#define GIMBAL_TASK_HPP

#include "controller/pids.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"
#include "tools/yaw_feedward/yaw_feedward.hpp"

constexpr float T_GIMBAL = 1e-3;

//遥控器模式云台数据
constexpr float W_MAX = 0.004f;  //rad/ms

// -------------------- 对外硬件 --------------------
inline sp::DM_Motor yaw_motor(0x08, 0x04, 3.141593f, 30.0f, 10.0f);

// -------------------- 对外调试 --------------------
extern float yaw_cmd_torque;
extern float yaw_target_angle;
extern float yaw_relative_angle;

#endif  // GIMBAL_TASK_HPP