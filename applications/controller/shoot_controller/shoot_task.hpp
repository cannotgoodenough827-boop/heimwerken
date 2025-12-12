#ifndef SHOOT_TASK_HPP
#define SHOOT_TASK_HPP
#include "motor/dm_motor/dm_motor.hpp"

inline sp::DM_Motor trigger_motor(0x01, 0x00, 3.1415926f, 30.0f, 10.0f);

#endif  // SHOOT_TASK_HPP