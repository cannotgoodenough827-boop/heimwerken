#ifndef GIMBAL_TASK_HPP
#define GIMBAL_TASK_HPP

#include "controller/pids.hpp"
#include "motor/cybergear_motor/cybergear_motor.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"
#include "tools/yaw_feedward/yaw_feedward.hpp"

// -------------------- 控制参数 --------------------
constexpr float T_GIMBAL = 1e-3;

#ifdef HERO_DOG
//遥控器模式云台数据
constexpr float W_MAX = 0.004f;  //rad/ms

constexpr float IMU_PITCH_ANGLE_MAX = 0.50f;   //Pitch轴限位    最大角度0.50
constexpr float IMU_PITCH_ANGLE_MIN = -0.25f;  //Pitch轴限位    最小角度-0.25
#endif

//重力补偿
// 云台平衡力矩系数N.M
#ifdef HERO_DOG
constexpr float TOR_PARAM = 1.1f;
// 重心偏角
constexpr float OFFSET_ANGLE = 0.0f;  // rad
#endif
#ifdef HERO_THREE_WHEELS
constexpr float TOR_PARAM = 1.4517f;
//重心偏角
constexpr float OFFSET_ANGLE = 0.0f;  // rad
#endif

// -------------------- 对外硬件 --------------------
inline sp::DM_Motor yaw_motor(0x08, 0x04, 3.141593f, 30.0f, 10.0f);
//气动0x06 0x00
inline sp::CyberGear_Motor pitch_motor(
  0x00, 0x01, CYBERGEAR_MAX_POSITION, CYBERGEAR_MAX_SPEED, CYBERGEAR_MAX_TORQUE);

// -------------------- 对外调试 --------------------
extern float yaw_target_angle;
extern float yaw_relative_angle;
extern float pitch_target_angle;
extern float pitch_relative_angle;

extern uint16_t gimbal_init_time;
extern uint16_t gimbal_init_over_time;
#endif  // GIMBAL_TASK_HPP