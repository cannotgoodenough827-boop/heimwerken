#include "uart_task.hpp"

extern "C" void uart_task()
{
  pm02.request();
  remote.request();

  while (true) {
    // 使用调试(f5)查看pm02内部变量的变化
    osDelay(10);
  }
}

extern "C" void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef * huart, uint16_t Size)
{
  auto stamp_ms = osKernelSysTick();

  if (huart == pm02.huart) {
    pm02.update(Size);
    pm02.request();
  }
  if (huart == &huart3) {
    remote.update(Size, stamp_ms);
    remote.request();
  }
}

extern "C" void HAL_UART_ErrorCallback(UART_HandleTypeDef * huart)
{
  if (huart == pm02.huart) {
    pm02.request();
  }
  if (huart == &huart3) {
    remote.request();
  }
}
