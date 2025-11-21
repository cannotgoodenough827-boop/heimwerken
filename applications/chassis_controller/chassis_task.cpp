#include "chassis_task.hpp"

#include "can/can.hpp"
#include "can/can_recv.hpp"
#include "cmsis_os.h"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "math.h"
#include "mode.hpp"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor/super_cap/super_cap.hpp"
#include "referee/pm02/pm02.hpp"
#include "referee/vt03/vt03.hpp"
#include "tools/pid/pid.hpp"
#include "uart/uart_task.hpp"

//功率控制
float infact_Pmax = 0.0f;
//底盘期望前后旋转速度
Chassis_Speed chassis_speed = {0.0f, 0.0f, 0.0f};
//经过偏移修正之后的轮子目标速度
Wheel_Speed chassis_target_speed = {0.0f, 0.0f, 0.0f, 0.0f};
//当前底盘四个电机轮子转速
Wheel_Speed wheel_speed = {0.0f, 0.0f, 0.0f, 0.0f};
//pid算出来的预期扭矩
Wheel_Torque wheel_give_torque = {0.0f, 0.0f, 0.0f, 0.0f};
sp::Mecanum chassis(WHEEL_RADIUS, (CHASSIS_LENGTH / 2), (CHASSIS_WIDTH / 2));

inline sp::PID speed_lf_pid(0.001f, 0.3f, 0.0f, 0.0f, 5.859f, 0.5f, 0.15f);
inline sp::PID speed_lr_pid(0.001f, 0.3f, 0.0f, 0.0f, 5.859f, 0.8f, 0.15f);
inline sp::PID speed_rf_pid(0.001f, 0.3f, 0.0f, 0.0f, 5.859f, 0.5f, 0.15f);
inline sp::PID speed_rr_pid(0.001f, 0.3f, 0.0f, 0.0f, 5.859f, 0.5f, 0.15f);

float remote_move_y = 0.0f;
float remote_move_x = 0.0f;

void chassis_control();
void chassis_mode_control();
void remote_speedcontrol_follow();

extern "C" void Chassis_task()
{
  can2.config();
  can2.start();
  while (1) {
    // 更新遥控器输入
    remote_move_x = remote.ch_lv;
    remote_move_y = remote.ch_rv;

    chassis_mode_control();
    // if(Chassis_Mode == CHASSIS_SPIN)
    //     {

    //     }
    if (Chassis_Mode == CHASSIS_MOVE) {
      remote_speedcontrol_follow();
    }
    chassis.calc(chassis_speed.vx, chassis_speed.vy, chassis_speed.wz);

    osDelay(1);
  }
}

void chassis_mode_control()
{
  switch (remote.sw_r) {
    case sp::DBusSwitchMode::DOWN:
      Chassis_Mode = CHASSIS_DOWN;
      break;

    case sp::DBusSwitchMode::MID:
      Chassis_Mode = CHASSIS_MOVE;
      break;

    case sp::DBusSwitchMode::UP:
      Chassis_Mode = CHASSIS_MOVE;
      break;
  }
}

void chassis_control()
{
  if (Chassis_Mode == CHASSIS_DOWN) {
    wheel_lf.cmd(0.0f);
    wheel_lr.cmd(0.0f);
    wheel_rf.cmd(0.0f);
    wheel_rr.cmd(0.0f);
    return;
  }
}

void remote_speedcontrol_follow()
{
  chassis_speed.vx = REMOTE_CONTROL_V * remote_move_x;
  chassis_speed.vy = -REMOTE_CONTROL_V * remote_move_y;
  speed_lf_pid.calc(chassis.speed_lf, wheel_lf.speed);
  speed_lr_pid.calc(chassis.speed_lr, wheel_lr.speed);
  speed_rf_pid.calc(chassis.speed_rf, wheel_rf.speed);
  speed_rr_pid.calc(chassis.speed_rr, wheel_rr.speed);
  wheel_lf.cmd(speed_lf_pid.out);
  wheel_lr.cmd(speed_lr_pid.out);
  wheel_rf.cmd(speed_rf_pid.out);
  wheel_rr.cmd(speed_rr_pid.out);
}