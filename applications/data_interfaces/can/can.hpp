#ifndef CAN_HPP
#define CAN_HPP
#include "io/can/can.hpp"

inline sp::CAN can1(&hcan1);
inline sp::CAN can2(&hcan2);

void chassis_send();

#endif  // CAN_HPP
