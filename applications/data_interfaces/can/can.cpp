#include "can.hpp"

#include "controller/chassis_controller/chassis_task.hpp"
#include "controller/gimbal_controller/gimbal_task.hpp"
#include "io/can/can.hpp"
#include "motor/rm_motor/rm_motor.hpp"
//chassis_CAN2
void chassis_send()
{
  wheel_lf.write(can2.tx_data);
  wheel_lr.write(can2.tx_data);
  wheel_rf.write(can2.tx_data);
  wheel_rr.write(can2.tx_data);

  can2.send(wheel_lf.tx_id);
}

void yaw_send()
{
  yaw_motor.write(can2.tx_data);
  can2.send(yaw_motor.tx_id);
}