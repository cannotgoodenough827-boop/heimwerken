#include "cmsis_os.h"
#include "io/servo/servo.hpp"

// C板
sp::Servo servo(&htim1, TIM_CHANNEL_1, 168e6f, 180.0f);  // 开发板最上面的PWM端口, 180度舵机

// 达妙
// sp::Servo servo(&htim1, TIM_CHANNEL_3, 240e6f, 180.0f); // 开发板最上面的PWM端口, 180度舵机

extern "C" void fpv_task()
{
  servo.start();

  while (true) {
    servo.set(90.0f);
    osDelay(10);
  }
}