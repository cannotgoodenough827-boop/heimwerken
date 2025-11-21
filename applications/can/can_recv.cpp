#include "can_recv.hpp"

#include "HERO_SELECTION.hpp"
#include "can.hpp"
#include "chassis_controller/chassis_task.hpp"
#include "cmsis_os.h"

extern "C" {
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef * hcan)
{
  auto stamp_ms = osKernelSysTick();
  while (HAL_CAN_GetRxFifoFillLevel(hcan, CAN_RX_FIFO0) > 0) {
    if (hcan == &hcan2) {
      can2.recv();
      if (can2.rx_id == wheel_lf.rx_id)
        wheel_lf.read(can2.rx_data, stamp_ms);
      else if (can2.rx_id == wheel_lr.rx_id)
        wheel_lr.read(can2.rx_data, stamp_ms);
      else if (can2.rx_id == wheel_rf.rx_id)
        wheel_rf.read(can2.rx_data, stamp_ms);
      else if (can2.rx_id == wheel_rr.rx_id)
        wheel_rr.read(can2.rx_data, stamp_ms);
      // else if (can2.rx_id == super_cap.rx_id)
      //   super_cap.read(can2.rx_data, stamp_ms);
    }
  }
}
}