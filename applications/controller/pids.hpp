#ifndef PIDS_HPP
#define PIDS_HPP
#include "HERO_SELECTION.hpp"
#include "tools/pid/pid.hpp"

constexpr float T_CONTROL = 1e-3f;  // 控制周期, 单位: s

inline sp::PID speed_lf_pid(0.001f, 0.3f, 0.0f, 0.0f, 5.859f, 0.5f, 0.15f);
inline sp::PID speed_lr_pid(0.001f, 0.3f, 0.0f, 0.0f, 5.859f, 0.8f, 0.15f);
inline sp::PID speed_rf_pid(0.001f, 0.3f, 0.0f, 0.0f, 5.859f, 0.5f, 0.15f);
inline sp::PID speed_rr_pid(0.001f, 0.3f, 0.0f, 0.0f, 5.859f, 0.5f, 0.15f);  //底盘电机速度pid

//gimbal pids
constexpr float MAX_4310_TORQUE = 10.0f;    // 达妙4310电机最大扭矩，单位N.m
constexpr float MAX_XIAOMI_TORQUE = 12.0f;  // 小米电机最大扭矩，单位N.m
#ifdef HERO_DOG
//位置控制PID
inline sp::PID yaw_pos_pid(T_CONTROL, 30.0f, 0.0f, 0.3f, 7, 3, 1.0f, true, false);
inline sp::PID yaw_speed_pid(
  T_CONTROL, 2.0f, 0.0f, 0.25f, MAX_4310_TORQUE, MAX_4310_TORQUE / 3.0f, 1.0f, false, false);
inline sp::PID pitch_pos_pid(T_CONTROL, 50.0f, 0.0f, 1.1f, 6, 0.18, 1.0f, true, false);
inline sp::PID pitch_speed_pid(
  T_CONTROL, 0.6f, 0.0f, 0.0f, MAX_XIAOMI_TORQUE, 0.2, 1.0f, false, false);

//初始化PID
inline sp::PID yaw_encode_pos_pid(T_CONTROL, 10.0f, 0.0f, 0.45f, 3, 1.5, 1.0f, true, false);
inline sp::PID yaw_encode_speed_pid(
  T_CONTROL, 1.5f, 0.0f, 0.12f, MAX_4310_TORQUE, MAX_4310_TORQUE / 3.0f, 1.0, false, false);

inline sp::PID pitch_encode_pos_pid(T_CONTROL, 13.0f, 100.0f, 0.05f, 5, 0.8, 1.0f, true, false);
inline sp::PID pitch_encode_speed_pid(
  T_CONTROL, 0.2f, 0.0f, 0.0f, MAX_XIAOMI_TORQUE, 0.1, 1.0f, false, false);

#endif
#endif  // PIDS_HPP
