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

//输入给yaw的扭矩
float yaw_cmd_torque = 0.0f;
float pitch_torque;
float gravity_compensation;
//达妙使能帧控制符
uint16_t yaw_enable_num = 0;
uint16_t pitch_enable_num = 0;

void motor_enable();
void chassis_control();
void gimbal_gyro_control();
void gimbal_encode_control();
void gimbal_control();
void yaw_init();
void pitch_init();
void yaw_error_clear();
void chassis_calculation_send();
Wheel_Torque chassis_pid_cal(float lf, float lr, float rf, float rr);

extern "C" void control_task()
{
  osDelay(500);
  can1.config();
  can1.start();
  can2.config();
  can2.start();
  yaw_init();  //使能
  pitch_init();
  while (1) {
    global_mode_control();
    motor_enable();
    yaw_error_clear();
    chassis_control();
    gimbal_control();
    gimbal_gyro_control();  //测试小米电机
    chassis_send();
    yaw_send();
    pitch_send();
    osDelay(1);
  }
}

void chassis_control()
{
  if (Chassis_Mode == CHASSIS_DOWN || Chassis_Mode == CHASSIS_INIT) {
    wheel_lf.cmd(0.0f);
    wheel_lr.cmd(0.0f);
    wheel_rf.cmd(0.0f);
    wheel_rr.cmd(0.0f);
  }
  chassis_calculation_send();
}

void chassis_calculation_send()
{
  //计算预期扭矩
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

Wheel_Torque chassis_pid_cal(float lf, float lr, float rf, float rr)  //传入目标速度
{
  Wheel_Torque wheel_given_torque_temp;  //结构体
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
    pitch_motor.cmd(0.0f);
  }
  //陀螺仪控制
  if (Gimbal_Mode == GIMBAL_GYRO) {
    gimbal_gyro_control();
  }
  if (Gimbal_Mode == GIMBAL_INIT) {
    gimbal_encode_control();
  }
}

void gimbal_encode_control()
{
  //解算ENCODE模式下两轴电流
  //yaw
  yaw_encode_pos_pid.calc(yaw_target_angle, yaw_relative_angle);
  yaw_encode_speed_pid.calc(yaw_pos_pid.out, imu_vyaw_filter);
  yaw_motor.cmd(yaw_encode_speed_pid.out);
  //pitch
  pitch_encode_pos_pid.calc(pitch_target_angle, pitch_relative_angle);
  pitch_encode_speed_pid.calc(pitch_encode_pos_pid.out, imu_vpitch_filter);
  gravity_compensation = cos(OFFSET_ANGLE + imu.pitch) * TOR_PARAM;
  pitch_torque = pitch_encode_speed_pid.out;  //+ gravity_compensation
  pitch_motor.cmd(pitch_torque);
}

void gimbal_gyro_control()
{
  //解算GYRO模式下两轴电流
  //yaw
  yaw_pos_pid.calc(yaw_target_angle, imu.yaw);
  yaw_speed_pid.calc(yaw_pos_pid.out, imu_vyaw_filter);
  yaw_cmd_torque = sp::limit_max(yaw_speed_pid.out, MAX_4310_TORQUE);
  yaw_motor.cmd(yaw_cmd_torque);
  //pitch
  pitch_pos_pid.calc(pitch_target_angle, imu.pitch);  //test
  pitch_speed_pid.calc(pitch_pos_pid.out, imu_vpitch_filter);
  gravity_compensation = cos(OFFSET_ANGLE + imu.pitch) * TOR_PARAM;
  pitch_torque = pitch_speed_pid.out + gravity_compensation;
  pitch_motor.cmd(pitch_torque);  //测试小米电机
}

//失能检测，发送使能帧
void motor_enable(void)
{
  if (yaw_motor.error == 0) {
    yaw_motor.write_enable(can2.tx_data);
    can2.send(yaw_motor.tx_id);
  }
  if (pitch_motor.error != 0) {
    pitch_init();
  }
  if (!yaw_motor_alive) {
    if (yaw_enable_num == 1000) {
      yaw_motor.write_enable(can2.tx_data);
      can2.send(yaw_motor.tx_id);
      yaw_enable_num = 0;
    }
    yaw_enable_num++;
  }
  //初次使能电机会进入Reset模式，需要再次使能设定为Motor模式
  if (!pitch_motor_alive || pitch_motor.mode != 2) {
    if (pitch_enable_num == 1000) {
      pitch_init();
      pitch_enable_num = 0;
    }
    pitch_enable_num++;
  }
}

void pitch_init()
{
  pitch_motor.cmd_set_single_parameter(run_mode, 0);  //设置为运控模式
  pitch_motor.write(can1.tx_data);
  can1.send_ext(pitch_motor.communication_type, 0, pitch_motor.motor_id, pitch_motor.master_id);
  osDelay(1);
  pitch_motor.cmd_motor_enable();  //电机使能
  pitch_motor.write(can1.tx_data);
  can1.send_ext(pitch_motor.communication_type, 0, pitch_motor.motor_id, pitch_motor.master_id);
}

void yaw_init()
{
  yaw_motor.write_enable(can2.tx_data);
  can2.send(yaw_motor.tx_id);
}

void yaw_error_clear()
{
  if (yaw_motor.error != 1 && yaw_motor.error != 0) {
    yaw_motor.write_clear_error(can2.tx_data);
    can2.send(yaw_motor.tx_id);
    yaw_motor.write_enable(can2.tx_data);
    can2.send(yaw_motor.tx_id);
  }
}