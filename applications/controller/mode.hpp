#ifndef MODE_HPP
#define MODE_HPP
#include "HERO_SELECTION.hpp"

typedef enum
{
  ZERO_FORCE,
  KEYBOARD,
  REMOTE,
} global_mode;

typedef enum
{
  CHASSIS_DOWN,
  CHASSIS_MOVE,
  CHASSIS_SPIN,
  CHASSIS_FOLLOW,
} chassis_mode;

typedef enum
{
  GIMBAL_ZERO_FORCE,  //无力状态
  GIMBAL_INIT,        //初始化回中
  GIMBAL_GYRO,        //非自瞄状态下陀螺仪控制
  GIMBAL_AUTO,        //自瞄状态下陀螺仪控制
} gimbal_mode;

void global_mode_control();
extern global_mode Global_Mode;
extern global_mode Last_Global_Mode;
extern chassis_mode Chassis_Mode;
//云台状态机
extern gimbal_mode Last_Gimbal_Mode;
extern gimbal_mode Gimbal_Mode;

#endif