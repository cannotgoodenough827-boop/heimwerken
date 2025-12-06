#include "calibrate_task.hpp"

uint8_t calibrate_flag;
uint16_t start_count;
uint16_t calibration_count;

//陀螺仪零飘数据
float gyro_x_zero;
float gyro_y_zero;
float gyro_z_zero;

float sum_x;
float sum_y;
float sum_z;

//初始化函数
void calibration_init(void);
//当摇杆"/\"",右拨杆在下时，开始校准，累计2s开始校准，校准开始有提示音，20s校准完毕同样有提示音
void calibration_start(void);
//陀螺仪均值校准
void gyro_calibration(void);

extern "C" void calibrate_task()
{
  osDelay(500);
  calibration_init();

  while (1) {
    calibration_start();
    gyro_calibration();
    osDelay(1);
  }
}

//初始化函数
void calibration_init(void)
{
  gyro_x_zero = GyroXZero;
  gyro_y_zero = GyroYZero;
  gyro_z_zero = GyroZZero;
  sum_x = 0.0f;
  sum_y = 0.0f;
  sum_z = 0.0f;
  calibrate_flag = 0;
  start_count = 0;
  calibration_count = 0;
}

//当摇杆"/\"",右拨杆在下时，开始校准，累计2s开始校准，校准开始有提示音，20s校准完毕同样有提示音
void calibration_start(void)
{
#ifdef DT7
  //当摇杆"/\"",右拨杆在下时
  if (
    (remote.ch_rh > 0.9f) && (remote.ch_rv < -0.9f) && (remote.ch_lh < -0.9f) &&
    (remote.ch_lv < -0.9f)) {
    //如果没有在校准
    if (calibrate_flag == 0) {
      if (start_count < 2000) {
        start_count++;
      }
      //校准开始
      if (start_count > 2000 - 1) {
        start_count = 0;
        calibrate_flag = 1;
        sum_x = 0.0f;
        sum_y = 0.0f;
        sum_z = 0.0f;
      }
    }
  }
#endif

#ifdef VT03
  //当摇杆"/\"",右拨杆在下时
  if (
    (Global_Mode == ZERO_FORCE) && (vt03.ch_rh > 0.9f) && (vt03.ch_rv < -0.9f) &&
    (vt03.ch_lh < -0.9f) && (vt03.ch_lv < -0.9f)) {
    //如果没有在校准
    if (calibrate_flag == 0) {
      if (start_count < 2000) {
        start_count++;
      }
      //校准开始
      if (start_count > 2000 - 1) {
        start_count = 0;
        calibrate_flag = 1;
        sum_x = 0.0f;
        sum_y = 0.0f;
        sum_z = 0.0f;
      }
    }
  }

#endif
}

//陀螺仪均值校准
void gyro_calibration(void)
{
  if (calibrate_flag == 1) {
    if (calibration_count < 20000) {
      sum_x += ins_gyro[0];
      sum_y += ins_gyro[1];
      sum_z += ins_gyro[2];
      calibration_count++;
      if (calibration_count == 20000) {
        gyro_x_zero += sum_x / 20000;
        gyro_y_zero += sum_y / 20000;
        gyro_z_zero += sum_z / 20000;
      }
    }
    else {
      calibrate_flag = 0;
      calibration_count = 0;
    }
  }
}
