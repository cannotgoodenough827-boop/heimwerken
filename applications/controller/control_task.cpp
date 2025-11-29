#include "control_task.hpp"

#include "HERO_SELECTION.hpp"
#include "cmsis_os.h"
#include "controller/chassis_controller/chassis_task.hpp"
#include "controller/detect_task.hpp"
#include "controller/gimbal_controller/gimbal_task.hpp"
#include "controller/mode.hpp"
#include "controller/pids.hpp"
#include "controller/power_control.hpp"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/can/can_recv.hpp"
#include "data_interfaces/uart/uart_task.hpp"
#include "io/imu_task.hpp"

uint16_t yaw_enable_num = 0;
void motor_enable();
void chassis_control();
void gimbal_gyro_control();
void gimbal_control();
Wheel_Torque chassis_pid_cal(float lf, float lr, float rf, float rr);

extern "C" void control_task()
{
  // osDelay(500);
  // can1.config();
  // can1.start();
  can2.config();
  can2.start();
  yaw_motor.write_enable(can2.tx_data);
  can2.send(yaw_motor.tx_id);
  while (1) {
    global_mode_control();
    motor_enable();

    if (yaw_motor.error != 1 && yaw_motor.error != 0) {
      yaw_motor.write_clear_error(can2.tx_data);
      can2.send(yaw_motor.tx_id);
      yaw_motor.write_enable(can2.tx_data);
      can2.send(yaw_motor.tx_id);
    }

    chassis_control();
    gimbal_control();
    chassis_send();
    yaw_send();
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
  chassis_power_control(
    &wheel_give_torque, &wheel_speed, &chassis_target_speed, infact_Pmax - 3.0f);
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

void gimbal_control()
{
  //云台模式为DOWN
  if (Gimbal_Mode == GIMBAL_ZERO_FORCE) {
    yaw_motor.cmd(0.0f);
  }
  if (Gimbal_Mode == GIMBAL_GYRO) {
    gimbal_gyro_control();
  }
}

void gimbal_gyro_control()
{
  //解算GYRO模式下两轴电流
  //yaw
  yaw_pos_pid.calc(yaw_target_angle, imu.yaw);
  yaw_speed_pid.calc(yaw_pos_pid.out, imu_vyaw_filter);
  yaw_cmd_torque = sp::limit_max(yaw_speed_pid.out, MAX_4310_TORQUE);
  yaw_motor.cmd(yaw_cmd_torque);
}

void motor_enable(void)
{
  if (yaw_motor.error == 0) {
    yaw_motor.write_enable(can2.tx_data);
    can2.send(yaw_motor.tx_id);
  }
  if (!yaw_motor_alive) {
    if (yaw_enable_num == 1000) {
      yaw_motor.write_enable(can2.tx_data);
      can2.send(yaw_motor.tx_id);
      yaw_enable_num = 0;
    }
    yaw_enable_num++;
  }
}