#include "cmsis_os.h"
#include "io/servo/servo.hpp"

// C板
sp::Servo servo(&htim1, TIM_CHANNEL_1, 168e6f, 180.0f);  // 开发板最上面的PWM端口, 180度舵机

extern "C" void fpv_task()
{
  servo.start();
  float angle = 0.0f;
  float speed = 45.0f;  // 每秒45°
  const float max_angle = 180.0f;
  const float dt = 0.01f;         // 每次循环间隔10ms
  const float step = speed * dt;  // 每次增加的角度
  bool s = true;

  while (s) {
    servo.set(angle);
    angle += step;
    if (angle >= max_angle) {
      s = false;
    }
    osDelay(10);
  }
}
