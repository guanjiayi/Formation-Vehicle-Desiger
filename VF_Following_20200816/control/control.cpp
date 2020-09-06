#include "control.hpp"
#include "../main/FV_Planner.hpp"

using namespace std;

Control::Control()
 {

 }
 
 Control::~Control()
{

}

void Control::SpeedControl()
{
    cout<<"SpeedControl Success"<<endl;
}

/************************************************
 名称：Control_update
 功能：控制线程执行函数
 输入：
 输出：

 ************************************************/
void Control::Control_update(){
    for(;;){
        control_mut.lock();
        //cout << "UWB_distance = " << dec << UWB_distance << endl;
        //cout << "UWB_fangwei = " << dec << UWB_fangwei  << endl;
        //cout << "UWB_zitai = " << dec << UWB_zitai << endl;
        mutex state_mut;
        state_mut.lock();
        Control_steer_enable = 1;

        // Record time
        /*time_t control_time;
        control_time = time(NULL);*/

        // convert from int to float
        float leader_speed = (float)(Leader_Speed) * 0.1;// m/s
        float follower_speed = (float)(Follower_Speed)*0.1; // m/s

        float follower_la_acc = (float)(Follower_La_acc)*0.1 - 15.0; //m/s^2
        float distance = (float)(UWB_distance)/100.0; // m
        float fangwei_angle = (float)(UWB_fangwei)/1.0; //degree
        float zitai_angle = (float)(UWB_zitai)/1.0; // degree

        //float leader_acc_pedal_position = (float)(Leader_ACC_pedal_position) * 0.4;
        //float leader_remote_position = (float)(Leader_Remote_position) * 0.4;
        //float leader_brake_pedal_position = (float)(Leader_Brake_pedal_position) * 0.01;
        float leader_actual_acc = (float)(Leader_Actual_acc)*0.1 - 15; //m/s^2
        //float leader_pressure = (float)(Leader_Pressure) * 0.01;
        //float leader_steering_wheel_angle = (float)(Leader_Steering_wheel_angle) * 0.1;
        //float leader_steering_wheel_speed = (float)(Leader_Steering_wheel_speed) * 4; //deg/s
        //float leader_steering_wheel_state = (float)(Leader_Steering_wheel_state) * ;
        //float leader_count = (float)(Leader_Count) * ;
        //float leader_check = (float)(Leader_Check) * ;
        //float leader_wheel_speed = (float)(Leader_Wheel_speed) * 0.01;
        //float leader_la_acc = (float)(Leader_La_acc) * 0.01; //m/s^2
        //float leader_yr_speed = (float)(Leader_Yr_speed) * 0.01; //rad/s
        //float leader_target_gear = (float)(Leader_Target_gear) * ;
        //float leader_current_gear = (float)(Leader_Current_gear) * ;
        // leader_acc_pedal = (float)(Leader_Acc_pedal);// %
        //float leader_brake_pedal = (float)(Leader_Brake_pedal); //%

        state_mut.unlock();

        // Caculate middle variables

        float long_distance;
        long_distance = distance * cos(fangwei_angle/180*M_PI); //m
        float lat_distance;
        lat_distance = distance * sin(fangwei_angle/180*M_PI); // m

        // 计算前轮转角（方向盘转角）
        float control_steer;        
        control_steer = Control::Caculate_steer(lat_distance,long_distance);// degree
        // 计算纵向加速度
        float control_acc;
        control_acc = Control::Caculate_acc(leader_speed,follower_speed,leader_actual_acc,distance); //m/s^2
        // 
        float control_brake_pressure;
        control_brake_pressure = 0.5;


        if(STATE_VALUE_PRINT|Show_switch){
           
            cout << "distance = " << distance << endl;
        }

        if(CONTROL_VALUE_PRINT|Show_switch){
            cout << "control acc = " << control_acc << endl;
            //cout << "control steer = " << control_steer << endl;
        }
        
        // 计算输出控制量
        Control_steer_angle = (int)((control_steer + 3276.7)/0.1); // Signal value = (physical value - offset)/precision value
        Control_acceleration = (int)((control_acc + 15)/0.1); //[-15,15] m/s^2
        Control_pressure = (int)((control_brake_pressure)/0.01); // [0,1]MPa
        control_mut.unlock();
        
        usleep(SAMPLE_TIME);
    }
}

/************************************************
 名称：Caculate_steer 
 功能：计算前轮转角
 输入：
 输出：

 ************************************************/
float Control::Caculate_steer(float lat_distance, float long_distance){
    //cout << "lat_distance = "<< to_string(lat_distance) << endl;
    float frontwheel_steer_angle = atan(2*L*lat_distance/(long_distance*long_distance))*180/M_PI;
    if(frontwheel_steer_angle > 15)
        frontwheel_steer_angle = 15;
    else if(frontwheel_steer_angle < -15)
        frontwheel_steer_angle = -15; // steer angle limit
    //cout << "Frontwheel_steer_angle = "<< to_string(frontwheel_steer_angle) << endl;
    float steer_wheel_angle = 24.1066 * frontwheel_steer_angle + 4.8505;// Caculate from steer map
    return steer_wheel_angle;
}
/************************************************
 名称：Caculate_acc
 功能：计算纵向加速度
 输入：
 输出：

 ************************************************/
float Control::Caculate_acc(float v1, float v2, float a1, float long_distance){
    float control_acc;
    if(Run_mode_switch == 0){
        /********Speed Keep control********/
        PID pid_acc(0.5,0.1,0);
        control_acc = pid_acc.pid_control(float(Desired_speed)/3.6,v2);
    }
    else{
        /******Distance Keeping Control*****/        
        
        float distance_error = long_distance - float(Desired_distance);

        if(distance_error < 10 && distance_error > -5)
            control_acc = k_a * a1 + k_v * (v1 - v2) + k_d * distance_error;
        else
            control_acc = k_a * a1/2 + k_v * (v1 - v2)/2 + k_d * distance_error * 2;

        if(control_acc < 0)
            control_acc = control_acc * 4;
        cout << "Leader Brake Pedal = " << Leader_Brake_pedal << endl;
        if(Leader_Brake_pedal > 45)
            control_acc = control_acc - 2.5;
        //control_acc = 0.05 * (long_distance - Desired_distance);
    }

    //Saturation
    if(control_acc > ACC_LIMIT)
        control_acc = ACC_LIMIT; // acc limit
    else if(control_acc < DEACC_LIMIT)
        control_acc = DEACC_LIMIT; //deacc limit
     // cout << "Control_acceleration = "<< to_string(control_acc) << endl;
    return control_acc;
}

/******************PID Control************************/
/************************************************
 名称：PID构造函数
 功能：初始PID参数
 输入：
 输出：

 ************************************************/
PID::PID():kp(0),ki(0),kd(0),target(0),actual(0),integral(0){
    error = target - actual;
    error_pre = error;
}

PID::PID(float p,float i,float d):kp(p),ki(i),kd(d),target(0),actual(0),integral(0){
    error = target - actual;
    error_pre = error;
}

/************************************************
 名称：pid_control
 功能：PID控制量求取
 输入：
tar 前车车速
cat 当前车速
 输出：
u 
 ************************************************/
float PID::pid_control(float tar,float act){
    float u;
    target = tar;
    actual = act;
    error = target - actual;
    integral += error;
    u = kp * error + ki * integral + kd * (error - error_pre);
    error_pre = error;
    return u;
}

/************************************************
 名称：pid_show
 功能：显示PID控制参数
 输入：
 输出：

 ************************************************/
void PID::pid_show(){
    cout << "Kp = " << kp << endl;
    cout << "Ki = " << ki << endl;
    cout << "Kd = " << kd << endl;
    cout << "Integral = " << integral << endl;
    cout << "Target = " << target << endl;
    cout << "Actual = " << actual << endl;
    cout << "Error = " << error << endl;
    cout << "Error_pre = " << error_pre << endl;
}