#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/pid/pid.hpp"

//实例化一个遥控器,串口为huart3
sp::DBus remote(&huart3);
//实例化can1,can2
sp::CAN can1(&hcan1);
sp::CAN can2(&hcan2);
//实例化4个M3508电机，id分别为1234
sp::RM_Motor motor_3508_1(1, sp::RM_Motors::M3508, 14.9f);
sp::RM_Motor motor_3508_2(2, sp::RM_Motors::M3508, 14.9f);
sp::RM_Motor motor_3508_3(3, sp::RM_Motors::M3508, 14.9f);
sp::RM_Motor motor_3508_4(4, sp::RM_Motors::M3508, 14.9f);
sp::PID motor1_pid_speed(0.001f, 0.2f, 0.01f, 0.0f, 50.0f, 50.0f, 1.0f, false, true);
sp::PID motor2_pid_speed(0.001f, 0.2f, 0.01f, 0.0f, 50.0f, 50.0f, 1.0f, false, true);
sp::PID motor3_pid_speed(0.001f, 0.2f, 0.01f, 0.0f, 50.0f, 50.0f, 1.0f, false, true);
sp::PID motor4_pid_speed(0.001f, 0.2f, 0.0f, 0.0f, 50.0f, 50.0f, 1.0f, false, true);

const float Lx = 0.185f;  // 底盘中心到轮子的X方向距离 (米)
const float Ly = 0.165f;  // 底盘中心到轮子的Y方向距离 (米)
const float R = 0.076f;   // 轮子半径 (米)，Mecanum轮直径≈152mm

float max_speed = 9.0f;
float Vx = 0.0f, Vy = 0.0f, Wz = 0.0f;
float w1 = 0.0f, w2 = 0.0f, w3 = 0.0f, w4 = 0.0f;
float rpm1 = 0.0f, rpm2 = 0.0f, rpm3 = 0.0f, rpm4 = 0.0f;

extern "C" void control_task()
{
  //接收遥控器消息
  remote.request();

  //can1初始化配置
  can2.config();
  can2.start();
  //ps：其他大多数电机cmd的均为扭矩值
  while (true) {
    switch (remote.sw_r) {
      case sp::DBusSwitchMode::UP:
        motor_3508_1.cmd(0.0f);
        motor_3508_2.cmd(0.0f);
        motor_3508_3.cmd(0.0f);
        motor_3508_4.cmd(0.0f);
        break;
      case sp::DBusSwitchMode::MID:
        Vx = remote.ch_lv * max_speed;  // 前后
        Vy = remote.ch_lh * max_speed;  // 左右

        if (fabs(remote.ch_rv) > 0.1f) {
          Wz = 2.0f;  // 左转
        }
        else if (fabs(remote.ch_rh) > 0.1f) {
          Wz = -2.0f;  // 右转
        }
        else {
          Wz = 0.0f;
        }
        //速度解算
        w1 = (-Vx + Vy + Wz * (Lx + Ly)) / R;  //左前,id2
        w2 = (-Vx - Vy + Wz * (Lx + Ly)) / R;  //右前,id1
        w3 = (Vx - Vy + Wz * (Lx + Ly)) / R;   //右后,id4
        w4 = (Vx + Vy + Wz * (Lx + Ly)) / R;   //左后,id3

        //转换到rpm
        rpm1 = w1 * 60.0f / (2.0f * 3.14f);
        rpm2 = w2 * 60.0f / (2.0f * 3.14f);
        rpm3 = w3 * 60.0f / (2.0f * 3.14f);
        rpm4 = w4 * 60.0f / (2.0f * 3.14f);

        motor1_pid_speed.calc(0.01*rpm2, motor_3508_1.speed);
        motor2_pid_speed.calc(0.01*rpm1, motor_3508_2.speed);
        motor3_pid_speed.calc(0.01*rpm4, motor_3508_3.speed);
        motor4_pid_speed.calc(0.01 * rpm3, motor_3508_4.speed);
        motor_3508_1.cmd(motor1_pid_speed.out);
        motor_3508_2.cmd(motor2_pid_speed.out);
        motor_3508_3.cmd(motor3_pid_speed.out);
        motor_3508_4.cmd(motor4_pid_speed.out);

        break;
        //约定右down挡时全部电机失能
      case sp::DBusSwitchMode::DOWN:
        motor_3508_1.cmd(0.0f);
        motor_3508_2.cmd(0.0f);
        motor_3508_3.cmd(0.0f);
        motor_3508_4.cmd(0.0f);
        break;
      default:
        break;
    }
    motor_3508_1.write(can2.tx_data);
    motor_3508_2.write(can2.tx_data);
    motor_3508_3.write(can2.tx_data);
    motor_3508_4.write(can2.tx_data);
    can2.send(motor_3508_1.tx_id);
    can2.send(motor_3508_2.tx_id);
    can2.send(motor_3508_3.tx_id);
    can2.send(motor_3508_4.tx_id);
    osDelay(10);
  }
}

//UART接收中断回调函数
extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

//UART错误中断回调函数
extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == &huart3) {
    remote.request();
  }
}

//CAN接收中断回调函数
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
    }
  }
}