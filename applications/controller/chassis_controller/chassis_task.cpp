#include "chassis_task.hpp"

#include "cmsis_os.h"
#include "controller/mode.hpp"
#include "controller/pids.hpp"
#include "data_interfaces/can/can.hpp"
#include "data_interfaces/can/can_recv.hpp"
#include "data_interfaces/uart/uart_task.hpp"
#include "io/can/can.hpp"
#include "io/dbus/dbus.hpp"
#include "math.h"
#include "motor/rm_motor/rm_motor.hpp"
#include "motor/super_cap/super_cap.hpp"
#include "referee/pm02/pm02.hpp"
#include "referee/vt03/vt03.hpp"
#include "tools/pid/pid.hpp"

//功率控制
float infact_Pmax = pm02.robot_status.chassis_power_limit;
//底盘期望前后旋转速度
Chassis_Speed chassis_speed = {0.0f, 0.0f, 0.0f};
//经过偏移修正之后的轮子目标速度
Wheel_Speed chassis_target_speed = {0.0f, 0.0f, 0.0f, 0.0f};
//当前底盘四个电机轮子转速
Wheel_Speed wheel_speed = {0.0f, 0.0f, 0.0f, 0.0f};
//pid算出来的预期扭矩
Wheel_Torque wheel_give_torque = {0.0f, 0.0f, 0.0f, 0.0f};

sp::Mecanum chassis(WHEEL_RADIUS, (CHASSIS_LENGTH / 2), (CHASSIS_WIDTH / 2));

void chassis_control();
//chassis_follow,spin下的遥控器/键鼠对应速度
void chassis_mode_control();
void remote_speedcontrol_follow();

extern "C" void Chassis_task()
{
  while (1) {
    chassis_mode_control();

    if (Chassis_Mode == CHASSIS_MOVE) {
      remote_speedcontrol_follow();
    }

    chassis.calc(chassis_speed.vx, chassis_speed.vy, chassis_speed.wz);
    wheel_speed.lf = wheel_lf.speed;
    wheel_speed.lr = wheel_lr.speed;
    wheel_speed.rf = wheel_rf.speed;
    wheel_speed.rr = wheel_rr.speed;

    chassis_target_speed.lf = chassis.speed_lf;
    chassis_target_speed.lr = chassis.speed_lr;
    chassis_target_speed.rf = chassis.speed_rf;
    chassis_target_speed.rr = chassis.speed_rr;

    osDelay(1);
  }
}

void chassis_mode_control()
{
  if (Global_Mode == ZERO_FORCE) {
    Chassis_Mode = CHASSIS_DOWN;
  }
  else {
#ifdef DT7
    if (Global_Mode == REMOTE) {
      if (remote.ch_lu < 0) {
        Chassis_Mode = CHASSIS_SPIN;
      }
      else if (remote.ch_lu > 0) {
        Chassis_Mode = CHASSIS_SPIN;
      }
      else {
        Chassis_Mode = CHASSIS_MOVE;
      }
    }
#endif
  }
}

void remote_speedcontrol_follow()
{
  chassis_speed.vx = REMOTE_CONTROL_V * remote.ch_lh;
  chassis_speed.vy = -REMOTE_CONTROL_V * remote.ch_lv;
  // chassis_speed.wz = SPIN_W * remote.ch_rh;
}