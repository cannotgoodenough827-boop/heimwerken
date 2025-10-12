#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "io/plotter/plotter.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor/super_cap/super_cap.hpp"
#include "referee/pm02/pm02.hpp"

sp::Plotter plotter(&huart1);

// extern sp::RM_Motor motor_3508_1;
// extern sp::RM_Motor motor_3508_2;
// extern sp::RM_Motor motor_3508_3;
// extern sp::RM_Motor motor_3508_4;
extern sp::PM02 pm02;
extern sp::SuperCap super_cap;
extern float g_P_in;
extern float g_P_real;
extern float P_max;


extern "C" void plot_task()
{
  while (true) {
    plotter.plot(g_P_in, g_P_real, pm02.robot_status.chassis_power_limit, pm02.power_heat.buffer_energy);
    osDelay(10);  // 100Hz
  }
}