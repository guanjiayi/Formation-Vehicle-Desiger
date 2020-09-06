#include "simulation.hpp"
#include "../main/FV_Planner.hpp"
#include <fstream>
#include <sstream>
#include <string.h>

Simulation::Simulation()
{
    //simth_flag=false;

}

Simulation::~Simulation()
{

}

void Simulation::SimUpdate()
{
    // 跟随车UWB信号
    int s_UWB_distance = 0 ;
    int s_UWB_fangwei = 0 ;
    int s_UWB_zitai = 0 ;
    int s_Follower_Speed = 0 ;
    int s_Follower_La_acc = 0 ;

    // 控制信号
    int s_Control_steer_angle = 0 ;
    int s_Control_steer_enable = 0 ;
    int s_Control_steer_velocity = 0 ;
    
    int s_Control_mode = 0;
    int s_Control_acceleration = 0;
    int s_Control_pressure = 0;

    // 互斥锁
    mutex SimLock;

    // 初始化读取流
    ifstream SimData_D;
    
    // 指定文件读取目录
    SimData_D.open("../simdata/foll_UWB.txt");

    //读取文件
    if(SimData_D.is_open())
    {
        string line=" ";

        while(!SimData_D.eof())
        {
            // 线程执行标志位
            //simth_flag=false;
            
            // 逐行读取
            getline(SimData_D,line);
            if(line.empty())
            {
                continue;                              
            }
            // 缓存
            istringstream istr(line);
            istr>>s_UWB_distance;
            istr>>s_UWB_fangwei;
            istr>>s_UWB_zitai;
            istr>>s_Follower_Speed;
            istr>>s_Follower_La_acc;  
            
            // 存储
            SimLock.lock();
            UWB_distance=s_UWB_distance;
            UWB_fangwei=s_UWB_fangwei;
            UWB_zitai=s_UWB_zitai;
            Follower_Speed=s_Follower_Speed;
            Follower_La_acc=s_Follower_La_acc;  
            SimLock.unlock();
            
            // 线程执行标志位
            //simth_flag=true;

            // 定周期读取
            usleep(20000);
        }
        SimData_D.close();
    }



}