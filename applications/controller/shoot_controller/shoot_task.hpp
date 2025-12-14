#ifndef SHOOT_TASK_HPP
#define SHOOT_TASK_HPP
#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"

inline sp::DM_Motor trigger_motor(0x01, 0x00, 3.1415926f, 30.0f, 10.0f);
inline sp::RM_Motor fric_motor1(1, sp::RM_Motors::M3508, 1);
inline sp::RM_Motor fric_motor2(2, sp::RM_Motors::M3508, 1);

#endif  // SHOOT_TASK_HPP