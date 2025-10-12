#include "cmsis_os.h"
#include "io/buzzer/buzzer.hpp"
constexpr int MOTOR_TIMEOUT = 500;  // 电机离线时间阈值, 单位: ms
// C板
sp::Buzzer buzzer(&htim4, TIM_CHANNEL_3, 84e6);

// 达妙
// sp::Buzzer buzzer(&htim12, TIM_CHANNEL_2, 240e6);
// 示例电机的上次更新时间（实际中你应该在CAN接收回调里更新这些）
extern uint32_t motor_3508_1_last_update;
extern uint32_t motor_3508_2_last_update;
extern uint32_t motor_3508_3_last_update;
extern uint32_t motor_3508_4_last_update;
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
// 当前系统时间
static inline uint32_t get_time_ms() { return osKernelSysTick(); }

extern "C" void buzzer_task()
{
  buzzer_power_on();

  // 演示两个不同报错旋律
  osDelay(1000);
  buzzer_error1();  // 模拟错误1
  osDelay(1000);
  buzzer_error2();  // 模拟错误2
  while (true) {
    uint32_t now = get_time_ms();

    bool motor_offline = false;

    if (now - motor_3508_1_last_update > MOTOR_TIMEOUT) motor_offline = true;
    if (now - motor_3508_2_last_update > MOTOR_TIMEOUT) motor_offline = true;
    if (now - motor_3508_3_last_update > MOTOR_TIMEOUT) motor_offline = true;
    if (now - motor_3508_4_last_update > MOTOR_TIMEOUT) motor_offline = true;

    if (motor_offline) {
      buzzer_error1();  // 电机离线报警
      osDelay(1000);    // 报警节奏间隔，防止连续响
    }
    else {
      osDelay(100);  // 定期检查
    }
    osDelay(100);
  }
}