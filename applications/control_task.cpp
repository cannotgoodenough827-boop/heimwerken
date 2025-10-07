#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "math.h"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor/super_cap/super_cap.hpp"
#include "referee/pm02/pm02.hpp"
#include "tools/pid/pid.hpp"
#define MOTOR_NUM 4

sp::DBus remote(&huart3);                            // 遥控器
sp::SuperCap super_cap(sp::SuperCapMode::AUTOMODE);  // 超级电容
sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);
sp::PM02 pm02(&huart6);
sp::RM_Motor motor_3508_1(1, sp::RM_Motors::M3508, 14.9f);
sp::RM_Motor motor_3508_2(2, sp::RM_Motors::M3508, 14.9f);
sp::RM_Motor motor_3508_3(3, sp::RM_Motors::M3508, 14.9f);
sp::RM_Motor motor_3508_4(4, sp::RM_Motors::M3508, 14.9f);

sp::PID motor1_pid_speed(0.001f, 0.2f, 0.0f, 0.0f, 50.0f, 50.0f, 1.0f, false, true);
sp::PID motor2_pid_speed(0.001f, 0.2f, 0.0f, 0.0f, 50.0f, 50.0f, 1.0f, false, true);
sp::PID motor3_pid_speed(0.001f, 0.2f, 0.0f, 0.0f, 50.0f, 50.0f, 1.0f, false, true);
sp::PID motor4_pid_speed(0.001f, 0.2f, 0.0f, 0.0f, 50.0f, 50.0f, 1.0f, false, true);

const float Lx = 0.185f;  // 底盘中心到轮子的X方向距离 (m)
const float Ly = 0.165f;  // 底盘中心到轮子的Y方向距离 (m)
const float R = 0.076f;   // 轮子半径 (m)

float max_speed = 10.0f;

// 功率限制相关参数
float P_max = 80;  // 最大功率（单位：J）
float K1 = 1.899f;
float K2 = 0.0055f;
float K3 = 3.8f;
float g_P_in;
float g_P_real;

void power_limit(float * tau, float * omega, uint8_t motor_num, float P_max)
{
  float sum_tau_omega = 0.0f;
  float sum_tau2 = 0.0f;
  float sum_omega2 = 0.0f;

  for (int i = 0; i < motor_num; i++) {
    sum_tau_omega += tau[i] * omega[i];
    sum_tau2 += tau[i] * tau[i];
    sum_omega2 += omega[i] * omega[i];
  }

  float P_in = sum_tau_omega + K1 * sum_tau2 + K2 * sum_omega2 + K3;  //J
  g_P_in = P_in;
  g_P_real = super_cap.power_in - super_cap.power_out;  // 实际功率（单位：J）
  //功率控制
  if (g_P_real > P_max) {
    P_max += super_cap.cap_energy;  //加上超级电容的能量
    float A = 2 * K1 * sum_tau2;
    float B = sum_tau_omega;
    float C = K2 * sum_omega2 + K3 - P_max;
    float discriminant = B * B - 4 * K1 * sum_tau2 * C;

    float K_scale = 1.0f;
    if (discriminant > 0 && A != 0) {
      K_scale = (-B + sqrtf(discriminant)) / A;
    }

    if (K_scale > 1.0f) K_scale = 1.0f;
    if (K_scale < -1.0f) K_scale = -1.0f;

    for (int i = 0; i < motor_num; i++) {
      tau[i] *= K_scale;
    }
  }
}

extern "C" void control_task()
{
  remote.request();
  can2.config();
  can2.start();

  float Vx = 0.0f, Vy = 0.0f, Wz = 0.0f;

  while (true) {
    switch (remote.sw_r) {
      case sp::DBusSwitchMode::UP:
        // motor_3508_1.cmd(0.0f);
        // motor_3508_2.cmd(0.0f);
        // motor_3508_3.cmd(0.0f);
        // motor_3508_4.cmd(0.0f);
        // sp::SuperCapMode::AUTOMODE;
        break;

      case sp::DBusSwitchMode::MID: {
        float w1 = 0.0f, w2 = 0.0f, w3 = 0.0f, w4 = 0.0f;

        Vx = remote.ch_lv * max_speed;
        Vy = remote.ch_lh * max_speed;
        Wz = remote.ch_rh * 2.0f;
        if (fabs(remote.ch_rv) > 0.1f) {
          Wz = remote.ch_rv * 8.0f;  // 左转
        }
        else if (fabs(remote.ch_rh) > 0.1f) {
          Wz = remote.ch_rh * 8.0f;  // 右转
        }
        else {
          Wz = 0.0f;
        }

        // 麦轮速度解算
        w1 = (-Vx + Vy + Wz * (Lx + Ly)) / R;  // 左前 2
        w2 = (-Vx - Vy + Wz * (Lx + Ly)) / R;  // 右前 1
        w3 = (Vx - Vy + Wz * (Lx + Ly)) / R;   // 右后 4
        w4 = (Vx + Vy + Wz * (Lx + Ly)) / R;   // 左后 3

        // PID 计算
        motor1_pid_speed.calc(w2, motor_3508_1.speed);
        motor2_pid_speed.calc(w1, motor_3508_2.speed);
        motor3_pid_speed.calc(w4, motor_3508_3.speed);
        motor4_pid_speed.calc(w3, motor_3508_4.speed);
        // motor_3508_1.cmd(motor1_pid_speed.out);
        // motor_3508_2.cmd(motor2_pid_speed.out);
        // motor_3508_3.cmd(motor3_pid_speed.out);
        // motor_3508_4.cmd(motor4_pid_speed.out);

        float tau[MOTOR_NUM] = {
          motor1_pid_speed.out, motor2_pid_speed.out, motor3_pid_speed.out, motor4_pid_speed.out};
        //电机实时转矩
        float omega[MOTOR_NUM] = {
          motor_3508_1.speed, motor_3508_2.speed, motor_3508_3.speed, motor_3508_4.speed};

        // // 功率计算
        // power_limit(tau, omega, MOTOR_NUM, pm02.robot_status.chassis_power_limit);
        power_limit(tau, omega, MOTOR_NUM, 80.0f);

        // 更新命令
        motor_3508_1.cmd(tau[0]);
        motor_3508_2.cmd(tau[1]);
        motor_3508_3.cmd(tau[2]);
        motor_3508_4.cmd(tau[3]);
        break;
      }
      // 右下拨杆：电机失能
      case sp::DBusSwitchMode::DOWN:
        sp::SuperCapMode::AUTOMODE;
        motor_3508_1.cmd(0.0f);
        motor_3508_2.cmd(0.0f);
        motor_3508_3.cmd(0.0f);
        motor_3508_4.cmd(0.0f);

        break;

      default:
        break;
    }

    // 写入电机数据到 CAN
    motor_3508_1.write(can2.tx_data);
    motor_3508_2.write(can2.tx_data);
    motor_3508_3.write(can2.tx_data);
    motor_3508_4.write(can2.tx_data);

    can2.send(motor_3508_1.tx_id);
    super_cap.write(
      can2.tx_data, pm02.robot_status.chassis_power_limit, pm02.power_heat.buffer_energy,
      pm02.robot_status.power_management_chassis_output);
    can2.send(super_cap.tx_id);

    osDelay(1);  // 100Hz
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
  if (huart == pm02.huart) {
    pm02.update(Size);
    pm02.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
  if (huart == pm02.huart) {
    pm02.request();
  }
}

extern "C" void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();

  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan2) {
      can2.recv();

      if (can2.rx_id == motor_3508_1.rx_id) motor_3508_1.read(can2.rx_data, stamp_ms);
      if (can2.rx_id == motor_3508_2.rx_id) motor_3508_2.read(can2.rx_data, stamp_ms);
      if (can2.rx_id == motor_3508_3.rx_id) motor_3508_3.read(can2.rx_data, stamp_ms);
      if (can2.rx_id == motor_3508_4.rx_id) motor_3508_4.read(can2.rx_data, stamp_ms);
      if (can2.rx_id == super_cap.rx_id) super_cap.read(can2.rx_data, stamp_ms);
    }
  }
}