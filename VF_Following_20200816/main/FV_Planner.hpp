#ifndef LEADERFOLLOWING_LEADER_FOLLOWING_H
#define LEADERFOLLOWING_LEADER_FOLLOWING_H

#endif //LEADERFOLLOWING_LEADER_FOLLOWING_H

#define SAMPLE_TIME 20000  //us
#define Sample_Time 20     //ms

//Switch whether to print
#define CAN_SEND_CHECK 0
#define CAN_RECEIVE_CHECK 0
#define STATE_VALUE_PRINT 1
#define CONTROL_VALUE_PRINT 1

// #define Simulink     // 判定是否为仿真线程

#include <thread>
#include <mutex>
#include <cmath>
#include <unistd.h>
#include <stdio.h>
#include <ctime>
#include <typeinfo>
#include <string>

using namespace std;

//Declare signals from follower
// 声明跟随车WUB信号
extern int UWB_distance;
extern int UWB_fangwei;
extern int UWB_zitai;

//extern int Leader_acceleration;
// 声明领航车加速度信号
extern int Follower_Speed;
extern int Follower_La_acc;

//Declare signals from leader
// 声明领航车信号变量
extern int Leader_ACC_pedal_position;
extern int Leader_Remote_position;
extern int Leader_Brake_pedal_position;
extern int Leader_Actual_acc;
extern int Leader_Speed;
extern int Leader_Pressure;
extern int Leader_Steering_wheel_angle;
extern int Leader_Steering_wheel_speed;
extern int Leader_Steering_wheel_state;
extern int Leader_Count;
extern int Leader_Check;
extern int Leader_Wheel_speed;
extern int Leader_La_acc;
extern int Leader_Yr_speed;
extern int Leader_Target_gear;
extern int Leader_Current_gear;
extern int Leader_Acc_pedal;
extern int Leader_Brake_pedal;

//Declare control signals output
// 声明控制信号变量
extern int Control_steer_enable;
extern int Control_steer_angle;
extern int Control_steer_velocity;

extern int Control_mode;       // 0:No brake;1:acc respond;2:pressure respond; 3: drive respond;
extern int Control_acceleration;
extern int Control_pressure;

//Declare Statemechine
// 声明状态变量
extern int State; // Current State
extern bool Signal_emergency;
extern bool Command_ready;
extern bool Command_run;
extern bool Command_finish;
extern bool Command_end;

extern int Desired_speed;
extern int Desired_distance;
extern bool Show_switch;
extern bool Run_mode_switch;
extern mutex control_mut;
extern mutex msg_mut;