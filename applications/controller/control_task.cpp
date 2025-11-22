#include "control_task.hpp"

#include "HERO_SELECTION.hpp"
#include "cmsis_os.h"
#include "controller/chassis_controller/chassis_task.hpp"
#include "controller/mode.hpp"
#include "controller/pids.hpp"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/can/can_recv.hpp"
#include "data_interfaces/uart/uart_task.hpp"

void chassis_control();
Wheel_Torque chassis_pid_cal(float lf, float lr, float rf, float rr);

extern "C" void control_task()
{
  osDelay(500);
  can1.config();
  can1.start();
  can2.config();
  can2.start();
  while (1) {
    global_mode_control();
    chassis_control();
    chassis_send();
    osDelay(1);
  }
}

void chassis_control()
{
  if (Chassis_Mode == CHASSIS_DOWN) {
    wheel_lf.cmd(0.0f);
    wheel_lr.cmd(0.0f);
    wheel_rf.cmd(0.0f);
    wheel_rr.cmd(0.0f);
  }
  wheel_give_torque = chassis_pid_cal(
    chassis_target_speed.lf, chassis_target_speed.lr, chassis_target_speed.rf,
    chassis_target_speed.rr);
  wheel_lf.cmd(wheel_give_torque.lf);
  wheel_lr.cmd(wheel_give_torque.lr);
  wheel_rf.cmd(wheel_give_torque.rf);
  wheel_rr.cmd(wheel_give_torque.rr);
}

Wheel_Torque chassis_pid_cal(float lf, float lr, float rf, float rr)
{
  Wheel_Torque wheel_given_torque_temp;
  speed_lf_pid.calc(lf, wheel_lf.speed);
  speed_lr_pid.calc(lr, wheel_lr.speed);
  speed_rf_pid.calc(rf, wheel_rf.speed);
  speed_rr_pid.calc(rr, wheel_rr.speed);
  wheel_given_torque_temp.lf = speed_lf_pid.out;
  wheel_given_torque_temp.lr = speed_lr_pid.out;
  wheel_given_torque_temp.rf = speed_rf_pid.out;
  wheel_given_torque_temp.rr = speed_rr_pid.out;
  return wheel_given_torque_temp;
}