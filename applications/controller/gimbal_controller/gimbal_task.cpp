#include "gimbal_task.hpp"

#include "HERO_SELECTION.hpp"
#include "cmsis_os.h"
#include "controller/detect_task.hpp"
#include "controller/mode.hpp"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/can/can_recv.hpp"
#include "data_interfaces/uart/uart_task.hpp"
#include "io/imu_task.hpp"
#include "motor/dm_motor/dm_motor.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "tools/low_pass_filter/low_pass_filter.hpp"
#include "tools/math_tools/math_tools.hpp"
#include "tools/mecanum/mecanum.hpp"

//变量们
//云台回中模式下回中后的时间
uint16_t gimbal_init_over_time = 0;
//云台进入回中模式的时间
uint16_t gimbal_init_time = 0;
//云台是否在回中模式
uint8_t gimbal_init_flag = 0;
//当中码盘值等效换算的角度
//区间：-Π~Π
float yaw_offecd_ecd_angle = 0.0f;
float pitch_offecd_ecd_angle = 0.0f;
//yaw解算的当前码盘值相对于正中码盘值的差
float yaw_relative_angle = 0.0f;
//pitch解算的当前码盘值相对于正中码盘值的差
float pitch_relative_angle = 0.0f;
//yaw，pitch目标参数
float gyro_yaw_angle_add = 0.0f;
float gyro_pitch_angle_add = 0.0f;
float yaw_target_angle = 0.0f;
float pitch_target_angle = 0.0f;

//云台初始化
void gimbal_init();
//云台状态选择
void gimbal_mode_control();
//云台电流解算
void gimbal_command();

extern "C" void gimbal_task()
{
  // 等待各个任务初始化完成
  osDelay(700);
  //云台初始化
  gimbal_init();
  while (1) {
    //云台电机选择模式
    gimbal_mode_control();
    gimbal_command();
    osDelay(1);
  }
}

void gimbal_init()
{
#ifdef HERO_DOG
  yaw_offecd_ecd_angle = -0.53349f;
  pitch_offecd_ecd_angle = 0.62146f;
#endif
#ifdef HERO_THREE_WHEELS
  yaw_offecd_ecd_angle = 2.3814f;
  pitch_offecd_ecd_angle = -0.780f;
#endif
}

void gimbal_mode_control()
{
  //正在回中过程中无法调整模式
  if (gimbal_init_flag == 1) {
    return;
  }
  if (Global_Mode == ZERO_FORCE) {
    Last_Gimbal_Mode = Gimbal_Mode;
    Gimbal_Mode = GIMBAL_ZERO_FORCE;
    return;
  }

  //遥控器模式
  if (Global_Mode == REMOTE) {
    Last_Gimbal_Mode = Gimbal_Mode;
    Gimbal_Mode = GIMBAL_GYRO;
  }
  //判断是否进入回中模式
  if (Last_Gimbal_Mode == GIMBAL_ZERO_FORCE && Gimbal_Mode != GIMBAL_ZERO_FORCE) {
    Gimbal_Mode = GIMBAL_INIT;
    gimbal_init_flag = 1;
  }
}

void gimbal_command()
{
  yaw_relative_angle = sp::limit_angle(yaw_motor.angle - yaw_offecd_ecd_angle);
  pitch_relative_angle = sp::limit_angle(pitch_motor.angle - pitch_offecd_ecd_angle);
  //云台模式为GYRO
  if (Gimbal_Mode == GIMBAL_GYRO) {
    //遥控器
    if (Global_Mode == REMOTE) {
      gyro_yaw_angle_add = -remote.ch_rh * W_MAX;
      gyro_pitch_angle_add = remote.ch_rv * W_MAX;
      yaw_target_angle = sp::limit_angle(yaw_target_angle + gyro_yaw_angle_add);
      pitch_target_angle = sp::limit_angle(pitch_target_angle + gyro_pitch_angle_add);
      //pitch轴限角
#ifdef RMUL
      pitch_target_angle =
        sp::limit_min_max(pitch_target_angle, IMU_PITCH_ANGLE_MIN, IMU_PITCH_ANGLE_MAX);
#endif
#ifdef RMUC
      pitch_target_angle = sp::limit_min_max(
        pitch_target_angle, IMU_PITCH_ANGLE_MIN + slope_angle, IMU_PITCH_ANGLE_MAX + slope_angle);
#endif
    }

    //pitch轴限角
#ifdef RMUL
    pitch_target_angle =
      sp::limit_min_max(pitch_target_angle, IMU_PITCH_ANGLE_MIN, IMU_PITCH_ANGLE_MAX);
#endif
#ifdef RMUC
    pitch_target_angle = sp::limit_min_max(
      pitch_target_angle, IMU_PITCH_ANGLE_MIN + slope_angle, IMU_PITCH_ANGLE_MAX + slope_angle);
#endif
  }
  if (Gimbal_Mode == GIMBAL_INIT) {
    yaw_target_angle = 0.0f;
    pitch_target_angle = 0.0f;
    if ((fabs(yaw_relative_angle)) < 0.05f && (fabs(pitch_relative_angle)) < 0.05f) {
      gimbal_init_over_time++;
    }
    gimbal_init_time++;

    //判断初始化完成
    if (gimbal_init_time == 1000 || gimbal_init_over_time == 500) {
      yaw_target_angle = imu.yaw;
      pitch_target_angle = imu.pitch;
      gimbal_init_over_time = 0;
      gimbal_init_time = 0;
      gimbal_init_flag = false;
    }
  }
}