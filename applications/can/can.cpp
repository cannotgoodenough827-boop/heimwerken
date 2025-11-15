#include "io/can/can.hpp"
//chassis_CAN2
void chassis_send()
{
  motor_3508_1.write(can2.tx_data);
  motor_3508_2.write(can2.tx_data);
  motor_3508_3.write(can2.tx_data);
  motor_3508_4.write(can2.tx_data);

  can2.send(motor_3508_1.tx_id);
}

void super_cap_send()
{
  super_cap.write(
    can2.tx_data, pm02.robot_status.chassis_power_limit, pm02.power_heat.buffer_energy,
    pm02.robot_status.power_management_chassis_output);
  can2.send(super_cap.tx_id);
}