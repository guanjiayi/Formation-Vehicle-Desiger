#include "../decision/decision.hpp"
#include "../canmessage/canmessage.hpp"
#include "../control/control.hpp"
#include "../planning/planning.hpp"
#include "../log/log.hpp"
#include "../simulation/simulation.hpp"
#include "FV_Planner.hpp"


// Define and initialize signals from follower
// 初始化跟随车UWB信号
int UWB_distance = 20 ;
int UWB_fangwei = 0 ;
int UWB_zitai = 0 ;

int Follower_Speed = 0 ;
int Follower_La_acc = 150 ;


//Define and initialize signals from leader
// 初始化领航车UWB信号
int Leader_ACC_pedal_position = 0;
int Leader_Remote_position = 0;
int Leader_Brake_pedal_position = 0;
int Leader_Actual_acc = 150;
int Leader_Speed = 0;

int Leader_Pressure = 0;
int Leader_Steering_wheel_angle = 0;    //
int Leader_Steering_wheel_speed = 0;
int Leader_Steering_wheel_state = 0;
int Leader_Count = 0;
int Leader_Check = 0;
int Leader_Wheel_speed = 0;
int Leader_La_acc = 0;
int Leader_Yr_speed = 0;
int Leader_Target_gear = 0;
int Leader_Current_gear = 0;
int Leader_Acc_pedal = 0;
int Leader_Brake_pedal = 0;

//Define and initialize control signals
// 初始化控制信号
int Control_steer_angle = 32767 ;
int Control_steer_enable = 0 ;
int Control_steer_velocity = 56 ;

int Control_mode = 2;
int Control_acceleration = 0;
int Control_pressure = 0.5;

//Define Statemechine
// 初始化状态信号
int State = 2;
bool Signal_emergency = 0;
bool Command_ready = 0;
bool Command_run = 0;
bool Command_finish = 0;
bool Command_end = 0;

int Desired_speed = 0;
int Desired_distance = 15;
bool Show_switch = 0;
bool Run_mode_switch = 1;

// 互斥锁
mutex control_mut;             // 控制线程互斥锁
mutex msg_mut;                 // CAN通信线程互斥锁

int main()
{
    CANMessage DataTransporter;     // 数据收发对象
    Decision Decider;               // 决策对象(状态机)
    Planning Planner;               // 规划对象（路径转换/路径规划）
    Control Controler;              // 控制对象（车速控制/方向控制）  
    Log Logger;                     // 数据记录对象
    Simulation Simulater;           // 仿真数据读取对象
   
  
   #ifdef  Simulink                  // 仿真过程
    // 加载仿真数据线程
    thread SimTh(&Simulater.SimUpdate);
     
   #else
   // 接收UWB位置信息线程
    thread RecivePosTh(&DataTransporter.CAN_receive,UWB_POSITION_MSG);       //receive UWB position (args:ID CANchannel)
    // 接收领航车状态信息线程
    thread ReciveLeaTh(&DataTransporter.CAN_receive,UWB_LEADERSTATE_MSG);    //receive UWB leader_state
    // 接收车辆ACC信息线程
    thread ReciveAccTh(&DataTransporter.CAN_receive,VEHICLE_ACC_MSG);        //receive follower ACC
    // 开启log数据记录线程
    thread LogTh(&Logger.LogUpdate);

   #endif

    // 决策线程（状态机）    
    thread DecisionTh(&Decider.state_transition);   
    // 开启路径规划线程（相对路径转换）      
    thread PlanningTh(&Planner.UpdatePlanning);
    // 控制线程
    thread ControlTh(&Controler.Control_update);                             //control signal update
    // 数据收发线程
    thread SendDataTh(&DataTransporter.CAN0_update);                         //send control signals


   #ifdef Simulink   
   // 仿真线程结束
   SimTh.join();

   #else
   // 数据接收线程结束
    RecivePosTh.join();
    ReciveLeaTh.join();
    ReciveAccTh.join();
    LogTh.join();
   #endif    
    
    // 决策线程结束
    DecisionTh.join();     
    // 控制线程结束
    ControlTh.join();
    // 控制数据发送线程
    SendDataTh.join();
    

    return 0;
         
}