#include "detect_task.hpp"

#include "HERO_SELECTION.hpp"
#include "cmsis_os.h"
#include "controller/chassis_controller/chassis_task.hpp"
#include "controller/gimbal_controller/gimbal_task.hpp"
#include "controller/mode.hpp"
#include "data_interfaces/uart/uart_task.hpp"
// #include "io/adc/adc.hpp"
#include "io/imu_task.hpp"
#include "math.h"
#include "stdio.h"
#include "stdlib.h"
#include "stm32f4xx_hal_cortex.h"

bool yaw_motor_alive = true;
bool pitch_motor_alive = false;
bool trigger_motor_alive = false;
bool chassis_alive = true;
bool gimbal_alive = true;
//坡角度
float slope_angle = 0.0f;
void motor_dead();

extern "C" void detect_task()
{
  while (1) {
    motor_dead();
    osDelay(10);
  }
}

void motor_dead()
{
  auto stamp_ms = osKernelSysTick();  // 获取当前的系统时间戳（以毫秒为单位）
  yaw_motor_alive = yaw_motor.is_alive(stamp_ms);
  pitch_motor_alive = pitch_motor.is_alive(stamp_ms);
  chassis_alive = wheel_lf.is_alive(stamp_ms) && wheel_lr.is_alive(stamp_ms) &&
                  wheel_rf.is_alive(stamp_ms) && wheel_rr.is_alive(stamp_ms);
  gimbal_alive = yaw_motor.is_alive(stamp_ms) && pitch_motor.is_alive(stamp_ms);
}