#include "log.hpp"
#include "../main/FV_Planner.hpp"
#include <fstream>

Log::Log()
{
    // bool logth_flg=false;

}

Log::~Log()
{

}

void Log::LogUpdate()
{
    // 跟随车UWB信号
    int l_UWB_distance = 0 ;
    int l_UWB_fangwei = 0 ;
    int l_UWB_zitai = 0 ;
    int l_Follower_Speed = 0 ;
    int l_Follower_La_acc = 0 ;

    // 控制信号
    int l_Control_steer_angle = 0 ;
    int l_Control_steer_enable = 0 ;
    int l_Control_steer_velocity = 0 ;
    
    int l_Control_mode = 0;
    int l_Control_acceleration = 0;
    int l_Control_pressure = 0;

    // 互斥锁
    mutex LogLock;

    // 指定存储文件位置
    ofstream Foll_UWB_D("../logdata/foll_UWB.txt");   //跟随车UWB
    ofstream Control_D("../logdata/Control.txt");     //控制数据  

    // Log线程循环主题
    while(1)
    {
        // 线程执行标志位至false
        //logth_flg=false;

        // 读取当前数据
        LogLock.lock();        
        l_UWB_distance = UWB_distance;
        l_UWB_fangwei = UWB_fangwei;
        l_UWB_zitai = UWB_zitai ;
        l_Follower_Speed = Follower_Speed ;
        l_Follower_La_acc = Follower_La_acc ;


        l_Control_steer_angle = Control_steer_angle ;
        l_Control_steer_enable = Control_steer_enable ;
        l_Control_steer_velocity = Control_steer_velocity ;
        l_Control_mode = Control_mode ;
        l_Control_acceleration = Control_acceleration;
        l_Control_pressure = Control_pressure;        
        LogLock.unlock();

        // 存储跟随车UWB数据
        Foll_UWB_D<< l_UWB_distance<<" "<<l_UWB_fangwei<<" "<<l_UWB_zitai
        <<" "<<l_Follower_Speed<<" "<<l_Follower_La_acc<<endl;

        // 存储控制数据
        Control_D<<l_Control_steer_angle<<" "<<l_Control_steer_enable<<" "<<l_Control_steer_velocity
        <<" "<<l_Control_mode<<" "<<l_Control_acceleration<<" "<<l_Control_pressure<<endl;
        
        // 线程执行标志位至true
        //logth_flg=true;
        
        // 线程等待20ms(线程周期)
        sleep(20);        

    }
    
    
}
