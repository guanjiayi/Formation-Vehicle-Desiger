#pragma once 
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
#include <vector>

using namespace std;

#define PointsNumber 200
// 底盘详细信息
struct ChassisDetail
{
    double leader_speed;
    double leader_acc;
    double leader_brake_pedal;
    double leader_acc_pedal;
    double steer_angle;
    double x_speed; 
    double x_acc ;
    double pedal_acc ;
    double pedal_brake ;
    int uwb_attitude ;
    int uwb_azimuth ;
    double uwb_distance ;
    int uwb_status ;
    double follower_yaw_rate ;
};

// 路径信息
struct TrajInfo
{
    double rel_x;
    double rel_y;
};

// 轨迹结构体信息
struct TrajSeq {
  float Seq[PointsNumber][5];  // x,y,rel_yaw,vx,yaw_rate
  vector<float> Seqv[5];
  int number;
  TrajSeq();
  void Print();  
};


//Declare signals from follower
// 声明跟随车WUB信号
extern int UWB_distance;                // 距离
extern int UWB_fangwei;                 // 方位角
extern int UWB_zitai;                   // 姿态角度

//extern int Leader_acceleration;
// 声明领航车加速度信号
extern int Follower_Speed;               // 跟随车车速
extern int Follower_La_acc;              // 跟随车纵向加速度    
extern double Follower_yaw_rate;         // 跟随车横摆角速度 edit justin 20200831
extern double Follower_steer_angle;      // 跟随车方向盘转角
extern double Follower_steer_speed;      // 跟随车方向盘转速

//Declare signals from leader
// 声明领航车信号变量
extern int Leader_ACC_pedal_position;    // 踏板角度
extern int Leader_Remote_position;
extern int Leader_Brake_pedal_position;
extern int Leader_Actual_acc;            // 前车加速度，需转换
extern int Leader_Speed;                 // 前车车速，需要转换
extern int Leader_Pressure;      
extern int Leader_Steering_wheel_angle;  // 方向盘转角
extern int Leader_Steering_wheel_speed;
extern int Leader_Steering_wheel_state;
extern int Leader_Count;
extern int Leader_Check;
extern int Leader_Wheel_speed;
extern int Leader_La_acc;
extern int Leader_Yr_speed;
extern int Leader_Target_gear;
extern int Leader_Current_gear;
extern int Leader_Acc_pedal;            // 踏板开度
extern int Leader_Brake_pedal;          // 制动开度

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
extern mutex PlanningLock;   // 规划互斥锁

extern bool PlanningThflag;  // 

// 前车轨迹信息
extern vector<TrajInfo>LeaderTraj;
