#ifndef UART_TASK_HPP
#define UART_TASK_HPP
#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "referee/pm02/pm02.hpp"
// -------------------- 对外接口 --------------------
inline sp::PM02 pm02(&huart6);    //裁判系统串口
inline sp::DBus remote(&huart3);  //遥控器串口

extern sp::DBus remote;

#endif