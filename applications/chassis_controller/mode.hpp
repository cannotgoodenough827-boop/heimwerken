#ifndef MODE_HPP
#define MODE_HPP

// typedef enum
// {
//   ZERO_FORCE,
//   REMOTE,
// } global_mode;

typedef enum
{
  CHASSIS_DOWN,
  CHASSIS_MOVE,
  CHASSIS_SPIN,
} chassis_mode;

//?????
extern chassis_mode Chassis_Mode;

#endif