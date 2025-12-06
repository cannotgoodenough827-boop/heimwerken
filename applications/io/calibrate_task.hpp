#ifndef _BUZZER_TASK_HPP_
#define _BUZZER_TASK_HPP_
#include <cstdint>

#include "HERO_SELECTION.hpp"
#include "cmsis_os.h"
#include "data_interfaces/uart/uart_task.hpp"
#include "imu_task.hpp"

// -------------------- ∂‘Õ‚µ˜ ‘ --------------------
extern uint8_t calibrate_flag;
extern float gyro_x_zero;
extern float gyro_y_zero;
extern float gyro_z_zero;

extern float sum_x;
extern float sum_y;
extern float sum_z;

#endif