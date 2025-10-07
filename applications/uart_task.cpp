#include "cmsis_os.h"
#include "referee/pm02/pm02.hpp"
extern sp::PM02 pm02;
extern "C" void uart_task()
{
  pm02.request();

  while (true) {
    // 使用调试(f5)查看pm02内部变量的变化
    osDelay(10);
  }
}