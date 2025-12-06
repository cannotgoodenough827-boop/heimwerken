#include "mode.hpp"

#include "cmsis_os.h"
#include "data_interfaces/uart/uart_task.hpp"

global_mode Global_Mode = ZERO_FORCE;
global_mode Last_Global_Mode = ZERO_FORCE;

chassis_mode Chassis_Mode;

//定义云台状态机
gimbal_mode Last_Gimbal_Mode;
gimbal_mode Gimbal_Mode;
//定义摩擦轮状态机
fric_mode Fric_Mode;
fric_mode Last_Fric_Mode;
//定义射击状态机
shoot_mode Shoot_Mode;
shoot_mode Last_Shoot_Mode;
//定义拨弹轮状态机
trigger_mode Trigger_Mode;
//记录反转之前的状态机
trigger_mode Last_Trigger_Mode;

//控制全局状态
void global_mode_control()
{
  Last_Global_Mode = Global_Mode;
  if (remote.sw_r == sp::DBusSwitchMode::DOWN) {
    Global_Mode = ZERO_FORCE;
    return;
  }
  if (remote.sw_r == sp::DBusSwitchMode::MID) Global_Mode = KEYBOARD;
  if (remote.sw_r == sp::DBusSwitchMode::UP) Global_Mode = REMOTE;
}