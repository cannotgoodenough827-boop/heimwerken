#include "cmsis_os.h"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
sp::Plotter plotter(&huart1);
extern sp::DBus remote;
extern sp::RM_Motor motor3508_1;
extern "C" void plotter_task()
{
  while (true) {
    plotter.plot(remote.ch_lh, motor3508_1.speed);
    osDelay(1);
  }
}