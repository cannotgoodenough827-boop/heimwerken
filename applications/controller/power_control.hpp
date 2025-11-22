#ifndef POWER_CONTROL_HPP_
#define POWER_CONTROL_HPP_
#include "HERO_SELECTION.hpp"
#include "controller/chassis_controller/chassis_task.hpp"

//由于infact_Pmax 确定原则：x<45/3(对抗赛的一级功率/3：虚弱时最小功率限制)；x-3(后面留的余量)>K3，防止功率控制无解。故K3不可超过15
#ifdef HERO_DOG
constexpr float K1 = 3.65f;
constexpr float K2 = 0.005f;
constexpr float K3 = 8.00f;
#endif

//完全基于误差分配
constexpr float ERROR_DISTRIBUTION_SET = 20.0f;
//完全基于功率分配
constexpr float POWER_DISTRIBUTION_SET = 15.0f;

// -------------------- 对外接口 --------------------
extern void chassis_power_control(
  Wheel_Torque * Data1, Wheel_Speed * Data2, Wheel_Speed * Data3, float P_max);
extern float infact_Pmax;
extern float P_in;
extern float Ewt, Ew2, Et2;
extern float wheel_speed_error[4];
//单个轮电机所需功率
extern float cmd_power[4];
//所需总功率
extern float sum_cmd_power;
//轮速小于目标速度的轮电机所需总功率
extern float sum_required_power;
//轮速小于目标速度的轮电机速度与目标速度总误差
extern float sum_wheel_error;
extern float error_confidence;
extern float allocatable_power;
extern float K_damping[4];
extern float K_damping_max;
extern float wheel_damping[4];

#endif