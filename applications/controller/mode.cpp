#include "mode.hpp"

#include "cmsis_os.h"
#include "data_interfaces/uart/uart_task.hpp"

global_mode Global_Mode = ZERO_FORCE;
global_mode Last_Global_Mode = ZERO_FORCE;

chassis_mode Chassis_Mode;

//定义云台状态机
gimbal_mode Last_Gimbal_Mode;
gimbal_mode Gimbal_Mode;

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