#ifndef MODE_HPP
#define MODE_HPP
#include "HERO_SELECTION.hpp"

typedef enum
{
  ZERO_FORCE,
  KEYBOARD,
  REMOTE,
} global_mode;

typedef enum
{
  CHASSIS_DOWN,
  CHASSIS_MOVE,
  CHASSIS_SPIN,
} chassis_mode;

void global_mode_control();
extern global_mode Global_Mode;
extern global_mode Last_Global_Mode;
extern chassis_mode Chassis_Mode;

#endif