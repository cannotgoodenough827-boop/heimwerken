#include "chassis_task.hpp"

#include "cmsis_os.h"
#include "controller/detect_task.hpp"
#include "controller/gimbal_controller/gimbal_task.hpp"
#include "controller/mode.hpp"
#include "controller/pids.hpp"
#include "controller/power_control.hpp"
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

//底盘初始化
bool chassis_init_flag = false;
uint32_t chassis_init_time = 0;
uint32_t chassis_init_over_time = 0;
//功率控制
float infact_Pmax = pm02.robot_status.chassis_power_limit;
//低压放电保护 0：电压低不可使用电容  1：可使用电容
uint8_t low_vol_flag = 1;
//底盘期望前后旋转速度
Chassis_Speed chassis_speed = {0.0f, 0.0f, 0.0f};
//经过偏移修正之后的轮子目标速度
Wheel_Speed chassis_target_speed = {0.0f, 0.0f, 0.0f, 0.0f};
//当前底盘四个电机轮子转速
Wheel_Speed wheel_speed = {0.0f, 0.0f, 0.0f, 0.0f};
//pid算出来的预期扭矩
Wheel_Torque wheel_give_torque = {0.0f, 0.0f, 0.0f, 0.0f};
//解算
sp::Mecanum chassis(WHEEL_RADIUS, (CHASSIS_LENGTH / 2), (CHASSIS_WIDTH / 2));
//小陀螺改变转向flag
bool spin_revert_flag;

void chassis_control();
//模式控制
void chassis_mode_control();
//remote控制底盘跟随
void remote_speedcontrol_follow();
//chassis_follow通用坐标系变至云台系+底盘跟随的函数
void chassis_coordinate_converter(Chassis_Speed * chassis_speed_given, float yaw_angle);
//底盘解算,待发
void chassis_command();

extern "C" void Chassis_task()
{
  while (1) {
    //总能量更新，还没写（）
    chassis_mode_control();

    if (chassis_init_flag) {
      if (chassis_alive) {
        chassis_init_over_time++;
      }
      chassis_init_time++;
      if (chassis_init_over_time == 100 || chassis_init_time == 7000) {
        chassis_init_over_time = 0;
        chassis_init_time = 0;
        chassis_init_flag = false;
      }
    }

    //底盘逆运动学，由四轮速度求解底盘速度
    chassis.update(wheel_lf.speed, wheel_lr.speed, wheel_rf.speed, wheel_rr.speed);
    chassis_command();
    osDelay(1);
  }
}

void chassis_mode_control()
{
  //底盘初始化优先级最高
  if (chassis_init_flag) {
    Chassis_Mode = CHASSIS_INIT;
    return;
  }
  if (Global_Mode == ZERO_FORCE || Gimbal_Mode == GIMBAL_INIT) {
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
        Chassis_Mode = CHASSIS_FOLLOW;
      }
    }
#endif
  }
}

void remote_speedcontrol_follow()
{
  chassis_speed.vx = REMOTE_CONTROL_V * remote.ch_lv;
  chassis_speed.vy = -REMOTE_CONTROL_V * remote.ch_lh;
  //底盘跟随角速度
  //平滑控制底盘跟随，解决启停扭动问题
  yaw_relative_angle_filter.update(yaw_relative_angle);
  yaw_relative_angle = yaw_relative_angle_filter.out;
  chassis_follow_wz_pid.calc(0.0f, yaw_relative_angle);  //底盘跟随：设为底盘与yaw轴相对角度为0
  chassis_speed.wz = -chassis_follow_wz_pid.out;
}

//chassis_follow通用坐标系变至云台系，解决小陀螺下平移的问题，以及底盘跟随状态下，移动以操作手视角移动
void chassis_coordinate_converter(Chassis_Speed * chassis_speed_given, float yaw_angle)
{
  //底盘解算
  float vx = chassis_speed_given->vx;
  float vy = chassis_speed_given->vy;
  chassis_speed_given->vx = vx * cos(yaw_angle) - vy * sin(yaw_angle);
  chassis_speed_given->vy = vx * sin(yaw_angle) + vy * cos(yaw_angle);
}

void chassis_command()
{
  if (Chassis_Mode == CHASSIS_FOLLOW) {
    if (Global_Mode == REMOTE) {
      remote_speedcontrol_follow();
    }
  }
  if (Chassis_Mode == CHASSIS_SPIN) {
    chassis_speed.wz = (spin_revert_flag ? -SPIN_W : SPIN_W);
  }
  
  //解算得到命令速度
  chassis_coordinate_converter(&chassis_speed, yaw_relative_angle);
  chassis.calc(chassis_speed.vx, chassis_speed.vy, chassis_speed.wz);
  wheel_speed.lf = wheel_lf.speed;
  wheel_speed.lr = wheel_lr.speed;
  wheel_speed.rf = wheel_rf.speed;
  wheel_speed.rr = wheel_rr.speed;

  chassis_target_speed.lf = chassis.speed_lf;
  chassis_target_speed.lr = chassis.speed_lr;
  chassis_target_speed.rf = chassis.speed_rf;
  chassis_target_speed.rr = chassis.speed_rr;
}

//根据剩余能量和电容情况确定最大功率
void Pmax_get()
{
#ifdef RMUC
  //电容低电保护
  if (low_vol_flag == 0 && super_cap.voltage >= 11.2) {
    low_vol_flag = 1;
  }
  if (low_vol_flag == 1 && super_cap.voltage < 6.2) {
    low_vol_flag = 0;
  }
  //剩余能量大于30%时 01111B = 16D
  if (pm02.buff.remaining_energy > 16) {
    infact_Pmax = pm02.robot_status.chassis_power_limit;
  }
  else {
    //剩余能量不足时限制最大功率为80W

    if (pm02.robot_status.chassis_power_limit > 80.0f) {
      infact_Pmax = 80.0f;
    }

    //根据电容的电压大小增加额外功率
    if (super_cap.voltage >= 6 && low_vol_flag == 1) {
      infact_Pmax = std::min(
        CAP_MAX_POWER, (super_cap.voltage - 6.0f) * 20 + pm02.robot_status.chassis_power_limit);
    }
    else {
      infact_Pmax = pm02.robot_status.chassis_power_limit;
    }
  }

#endif
  buff_energy_p_limited();
}

void buff_energy_p_limited()
{
  if (pm02.power_heat.buffer_energy < 20.0f) {
    //infact_Pmax 确定原则：x<45/3(对抗赛的一级功率/3：虚弱时最小功率限制)；x-3(后面留的余量)>K3，防止功率控制无解
    infact_Pmax = K3 + 4.0f;
  }
}