#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"

sp::Plotter plotter(&huart1);

extern sp::RM_Motor motor_3508_1;
extern sp::RM_Motor motor_3508_2;
extern sp::RM_Motor motor_3508_3;
extern sp::RM_Motor motor_3508_4;
extern "C" void plot_task()
{
  while (true) {
    plotter.plot(motor_3508_1.speed, motor_3508_2.speed, motor_3508_3.speed, motor_3508_4.speed);
    osDelay(10);  // 100Hz
  }
}