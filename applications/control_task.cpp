#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "motor/rm_motor/rm_motor.hpp"

//实例化一个遥控器
sp::DBus remote(&huart3);

//实例化can1
sp::CAN can1(&hcan1);
//实例化一个id为1的电压环（6020_V）6020电机
sp::RM_Motor motor_3508(1, sp::RM_Motors::M3508, 14.9f);

extern "C" void control_task()
{
  //接收遥控器消息
  remote.request();

  //can1初始化配置
  can1.config();
  can1.start();
  //遥控器右边拨杆上中下挡控制输入给电机的电压值分别为5.5V, 2.0V, 0V
  //ps：其他大多数电机cmd的均为扭矩值
  while (true) {
    switch (remote.sw_r) {
      case sp::DBusSwitchMode::UP:
        motor_6020.cmd(5.5f);
        break;
      case sp::DBusSwitchMode::MID:
        motor_6020.cmd(2.0f);
        break;
        //约定右down挡时全部电机失能
      case sp::DBusSwitchMode::DOWN:
        motor_6020.cmd(0.0f);
        break;
      default:
        break;
    }
    motor_6020.write(can1.tx_data);
    can1.send(motor_6020.tx_id);
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
    if (hcan == &hcan1) {
      can1.recv();

      if (can1.rx_id == motor_6020.rx_id) motor_6020.read(can1.rx_data, stamp_ms);
    }
  }
}