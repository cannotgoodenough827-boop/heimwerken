#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor/super_cap/super_cap.hpp"

sp::Plotter plotter(&huart1);

// extern sp::RM_Motor motor_3508_1;
// extern sp::RM_Motor motor_3508_2;
// extern sp::RM_Motor motor_3508_3;
// extern sp::RM_Motor motor_3508_4;
extern sp::SuperCap super_cap;
extern float g_P_in;
extern float g_P_real;
extern float P_max;

extern "C" void plot_task()
{
  while (true) {
    plotter.plot(g_P_in, g_P_real, P_max, super_cap.cap_energy);
    osDelay(10);  // 100Hz
  }
}