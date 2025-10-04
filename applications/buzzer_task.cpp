#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"

// C板
sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

// 达妙
// sp::Buzzer buzzer(&htim12, TIM_CHANNEL_2, 240e6);

void buzzer_power_on()
{
  buzzer.set(5000, 0.1);

  for (int i = 0; i < 3; i++) {
    buzzer.start();
    osDelay(100);
    buzzer.stop();
    osDelay(100);
  }
}
void buzzer_error1()

{
  int freq[3] = {6000, 4000, 6000};  // 高频 → 中频 → 高频
  for (int i = 0; i < 3; i++) {
    buzzer.set(freq[i], 0.2);
    buzzer.start();
    osDelay(150);
    buzzer.stop();
    osDelay(100);
  }
}

void buzzer_error2()
{
  buzzer.set(2000, 0.3);  // 低音
  buzzer.start();
  osDelay(300);
  buzzer.stop();
  osDelay(100);

  buzzer.set(5000, 0.3);  // 高频长音
  buzzer.start();
  osDelay(600);
  buzzer.stop();
}

extern "C" void buzzer_task()
{
  buzzer_power_on();

  // 演示两个不同报错旋律
  osDelay(1000);
  buzzer_error1();  // 模拟错误1
  osDelay(1000);
  buzzer_error2();  // 模拟错误2
  while (true) {
    osDelay(100);
  }
}