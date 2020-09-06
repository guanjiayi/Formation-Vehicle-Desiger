#include "control.hpp"
#include "../main/FV_Planner.hpp"
#include <string.h>

using namespace std;

constexpr float L = 5.0;
constexpr float r = 1.0;
constexpr float k_a = 0.1;
constexpr float k_v = 0.05;
constexpr float k_d = 0.05;

Control::Control()
 {

 }
 
 Control::~Control()
{

}

/************************************************
 名称：Control_update
 功能：控制线程执行函数
 输入：
 输出：

 ************************************************/
void Control::UpdateControl(){

    // 变量交互与数据预处理
    ChassisDetail ChassisData;     // 位姿信息
    // 互斥锁
    mutex ControlLock;
    
    // 贝赛尔曲线点数组至零
    memset(&BezierX,0,sizeof(BezierX));
    memset(&BezierY,0,sizeof(BezierY));

    usleep(SAMPLE_TIME);

    while(1)
    {
      Control_steer_enable = 1;

      if(PlanningThflag)
      {
        ControlLock.lock();
        // 领航车信息
        ChassisData.leader_speed=(double)Leader_Speed*0.1;                        // 引导车车速 m/s
        ChassisData.leader_acc=(double)Leader_Actual_acc*0.1-15;                    // 引导车加速度 m/s^2
        //ChassisData.leader_brake_pedal=(double)Leader_Brake_pedal_position*0.01;  // 引导车制动踏板开度 %
        //ChassisData.leader_acc_pedal=(double)Leader_ACC_pedal_position*0.4;       // 引导车加速踏板开度 %

        ChassisData.leader_brake_pedal=(double)(Leader_Brake_pedal)*0.01;           // 引导车制动踏板开度 [0-100]
        ChassisData.leader_acc_pedal=(double)(Leader_Acc_pedal)*0.01;               // 引导车加速踏板开度 [0-100]
        // 
        // ChassisData.steer_angle=(double)Leader_Steering_wheel_angle*0.1-3276.7;     // 引导车方向盘转角
        ChassisData.steer_angle=(double)Follower_steer_angle*0.1;                  // 跟随车方向盘转角
        ChassisData.x_speed=(double)Follower_Speed*0.1;                             // 跟随车车速 m/s
        // ChassisData.x_speed=1.2;
        ChassisData.x_acc=(double)(Follower_La_acc)*0.1-15.0;                       // 跟随车纵向加速度 m/s^2
        ChassisData.pedal_acc=0;                                                    // 跟随车加速踏板开度%
        ChassisData.pedal_brake=0;                                                  // 跟随车制动踏板开度%
        ChassisData.uwb_attitude=(double)UWB_zitai;                                 // deg
        ChassisData.uwb_azimuth=(double)UWB_fangwei;                                // deg
        ChassisData.uwb_distance=(double)(UWB_distance)/100;                        // m
        ChassisData.uwb_status=Leader_Check;                                        // 
        ChassisData.follower_yaw_rate=Follower_yaw_rate;                            // deg/s

        vector<TrajInfo>LeaderPath;    // 前导车轨迹
        // 复制轨迹曲线
        LeaderPath.clear();     
        LeaderPath.assign(LeaderTraj.begin(),LeaderTraj.end());

        ControlLock.unlock();

        cout<<"number:"<<LeaderPath.size()<<endl;

        // cout<< "LeaderPath:"<<LeaderPath.size()<<endl;
        //cout<<"LeaderPath.x: "<<LeaderPath[50].rel_x<<endl;
        //cout<<"LeaderPath.y: "<<LeaderPath[50].rel_y<<endl;
        //cout<<"steer_angle="<<ChassisData.steer_angle<<endl;
        cout<<"brake_pedal= "<<ChassisData.leader_brake_pedal<<endl;
        cout<<"acc_pedal="<<ChassisData.leader_acc_pedal<<endl;

        float control_steer=0;

       // 如果路点加载出现问题则采用纯追踪计算航向
       if(LeaderPath.size()<2)
       {
         control_steer=Caculate_steer_pure(ChassisData);
       }
       else
       {
          // 平滑处理前车轨迹
          DealTraj(LeaderPath);      
          // 计算方向盘转角
          control_steer=Caculate_steer(ChassisData,LeaderPath);
          cout<<"steer control sucess!"<<endl;
          
       }
            
        // 计算期望加速度
        float control_acc = Caculate_acc(ChassisData);
        
        // 打印加速度和方向盘转角
        cout<<"control_acc:"<<control_acc<<endl;
        cout<< "control_steer:"<<control_steer<<endl;
        

        float control_brake_pressure=5; 

        ControlLock.lock();
        Control_steer_angle = (int)((control_steer + 3276.7)/0.1); // Signal value = (physical value - offset)/precision value
        Control_acceleration = (int)((control_acc + 15)/0.1);      // [-15,15] m/s^2
        Control_pressure = (int)((control_brake_pressure)/0.01);   // [0,1]MPa
        ControlLock.unlock();

        usleep(SAMPLE_TIME);
      }
      else 
      {
        usleep(SAMPLE_TIME/2);
        continue;
      }
      
    }
   
}

/************************************************
 名称：
 功能：
 输入：
 输出：
 ************************************************/
void Control::DealTraj(const vector<TrajInfo> OrgTraj) {
 
  // Bezier Curve 
  // 贝赛尔曲线平滑
   BezierFitting(OrgTraj);

  // Lateral error 
  // 查找车头2m的ID索引
  int index_la = FindLookAheadPointBezier(2);
  // 车头误差
  err_lat = BezierX[index_la];
  
}

/************************************************
 名称：BezierFitting
 功能：
 输入：
 输出：
 ************************************************/
void Control::BezierFitting(const vector<TrajInfo> msg1) {
  
  // 清零
  for (int i = 0; i <= 250; i++) {
    BezierX[i] = BezierY[i] = 0;
  }
  
  int number = msg1.size();
  // cout<<"trajnumber:"<<number<<endl;
  // 4 control point
  int ControlPointIndex[6];            // 控制点ID
  float Px[6]={0}, Py[6]={0};          // 控制点坐标

  
  // 选择控制点
  if(number>1)
  {
    for (int i = 0; i < 6; i++) {
    ControlPointIndex[i] = floor((number - 1) * i / 5);
    Px[i] = msg1[ControlPointIndex[i]].rel_x;
    Py[i] = msg1[ControlPointIndex[i]].rel_y;
    
    }
  }
  
  // Cal Curve
  // 生成贝塞尔曲线点
  for (int i = 0; i < 251; i++) {
    BezierX[i] = CalBezierLoc(6, i * 0.004, Px);
    BezierY[i] = CalBezierLoc(6, i * 0.004, Py);
  }

}

/***********************************TrajInfo*************
 名称：
 功能：
 输入：
 输出：
 ************************************************/
float Control::CalBezierLoc(int n, float t, float p[]) {
  float loc = 0;
  
  for (int j = 0; j < n; j++) {
    float ret = 1;float Desired_speed = 0;
float Desired_distance = 15;
    for (int i = 0; i < j; i++) ret *= t;
    for (int i = 0; i < n - 1 - j; i++) ret *= (1 - t);
    // nchoosek
    for (int i = n - 1; i >= n - j; i--) ret *= (i);
    for (int i = 1; i <= j; i++) ret /= i;
    ret *= p[j];
    loc += ret;
  }

  return loc;
}

/*
  Input ChassisDetail message
  Output Control_steer degree
*/
/************************************************
 名称：
 功能：
 输入：
 输出：
 ************************************************/
float Control::Caculate_steer(const ChassisDetail msg0, const vector<TrajInfo> msg1) 
{
    
  // /* Specify a lookahead point*/
  // int index_pro = FindLookAheadPointBezier(10);
  
  // cout<<"index_la:"<<index_pro<<endl;
  // float long_distance;
  // long_distance = BezierX[index_pro];
  // cout << "long_distance:"<<long_distance<<endl;

  // // 初始化PID 对象
  // PID pid_steer(steer_PID_kp, steer_PID_ki, steer_PID_kd);

  // float lat_distance;
  // lat_distance = BezierY[index_pro];
  // cout<<"lat_distance:"<<lat_distance<<endl;

  // float frontwheel_steer_angle =
  //      -0.6* atan(2 * L * lat_distance / (long_distance * long_distance)) * 180 /M_PI +
  //       0.4*pid_steer.pid_control(0, err_lat) ;

  // 中间变量初始化
  int index_pro=0;                 // 预瞄点ID
  int index_cur=0;                 // 当前点ID
  float pro_long_distance=1;       // 预瞄点纵向坐标
  float pro_lat_distance=0;        // 预瞄点横向坐标
  float cur_lat_distance=0;        // 当前点横向坐标
  
  float k_steer;                   // 纯追踪比例系数
  k_steer=0.9;

  // 初始化PID 对象
  PID pid_steer(0.1, 0, 0.002);
 
  // 计算预瞄点就当前点ID
  index_pro=FindBezierPointID(12);
  index_cur=FindBezierPointID(2.5);

  // 计算预瞄点偏差
  pro_long_distance=BezierX[index_pro];
  pro_lat_distance=BezierY[index_pro];

  // 计算当前点偏差
  cur_lat_distance=BezierY[index_cur];

  cout<<"pro_long_distance: "<<pro_long_distance<<endl;
  cout<<"pro_lat_distance: "<<pro_lat_distance<<endl;
  cout<<"cur_lat_distance: "<<cur_lat_distance<<endl;

  // float frontwheel_steer_angle1=-3*lat_distance;
  // float frontwheel_steer_angle2=-1*err_lat;
  // float frontwheel_steer_angle=0.8*frontwheel_steer_angle1+0.2*frontwheel_steer_angle2;
 
  // 计算转向轮转角
  float frontwheel_steer_angle =
         k_steer*(-1)* atan(2 * L * pro_lat_distance / (pro_long_distance * pro_long_distance)) * 180 /M_PI +
         (1-k_steer)*pid_steer.pid_control(0, cur_lat_distance) ;
  
  // 前轮转角限值
  if (frontwheel_steer_angle > 20)
  {
    frontwheel_steer_angle = 20;
  }
  else if (frontwheel_steer_angle < -20)
  {
    frontwheel_steer_angle = -20;  

  }
    
  // 前轮转角转换成方向盘转角
  float steer_wheel_angle = 24.1066 * frontwheel_steer_angle + 4.8505;  // Caculate from steer map
  
  // 返回方向盘转角
  return steer_wheel_angle;
}

/************************************************
 名称：Caculate_steer_pure
 功能：根据纯追踪计算方向盘转角
 输入：
 mg0 
 输出：
 steer_wheel_angle 方向盘转角
 ************************************************/
float Control::Caculate_steer_pure(const ChassisDetail msg0)
{
  float distance =msg0.uwb_distance;
  float fangwei_angle=msg0.uwb_azimuth;
  
  float long_distance=0;
  float lat_distance=0;
  float frontwheel_steer_angle=0;

  long_distance=distance*cos(fangwei_angle/180*M_PI); 
  lat_distance=distance*sin(fangwei_angle/180*M_PI); 

  // 计算方向盘转角
  frontwheel_steer_angle= atan(2*L*lat_distance/(long_distance*long_distance))*180/M_PI;

  // 前轮转角限值
  if(frontwheel_steer_angle > 20)
  {
    frontwheel_steer_angle = 20;
  }
  else if(frontwheel_steer_angle < -20)
  {
    frontwheel_steer_angle = -20;     // steer angle limit
  }
  
  // 前轮转角转化为方向盘转角
  float steer_wheel_angle = 24.1066 * frontwheel_steer_angle + 4.8505;// Caculate from steer map
  
  // 返回方向盘转角
  return steer_wheel_angle;
}

/*
  Input ChassisDetail message
  Output Control_acc m/s^2
*/
/************************************************
 名称：
 功能：
 输入：
 输出：
 ************************************************/
float Control::Caculate_acc(const ChassisDetail msg0) {
  float v1 = msg0.leader_speed;                    // 引导车车速
  float v2 = msg0.x_speed;                         // 跟随车车速
  float a1 = msg0.leader_acc;                      // 引导车加速度
  float distance = msg0.uwb_distance;              // UWB跟车距离
  float azimuth_angle = msg0.uwb_azimuth;          // UWB方位角
  float long_distance = distance * cos(azimuth_angle / 180 * M_PI);  // 纵向跟车距离
  float Leader_Brake_pedal = msg0.leader_brake_pedal;        // 前车制动踏板开度    
  float Leader_Acc_pedal=msg0.leader_acc_pedal;              // 前车加速度踏板开度
  float control_acc = 0;

  /******Distance Keeping Control*****/

  //float distance_error = long_distance - float(Desired_distance);

  float distance_error = distance - float(Desired_distance);     // 纵向距离跟踪误差
  
  // 根据相对车距和相对车速计算纵向跟车策略
  if (distance_error < 10 && distance_error > -5)
  {
    control_acc = k_a * a1 + k_v * (v1 - v2) + k_d * distance_error;
  }
  else
  {
    control_acc = k_a * a1 / 2 + k_v * (v1 - v2) / 2 + k_d * distance_error * 2;
  } 
      
  // 制动是加大制动力
  if (control_acc < 0) 
  {
    control_acc = control_acc * 4;
  }
  

  // cout << "Leader Brake Pedal = " << Leader_Brake_pedal << endl;
  //if (Leader_Brake_pedal > 45) control_acc = control_acc - 2.5;

  // 纵向控制前馈
  if(Leader_Brake_pedal>0.2)
  {
    double pedal=Leader_Brake_pedal/100;
    if(pedal>0.1 && pedal <0.5) 
    {
      double k=3*(0.6+0.8*pedal);
      control_acc=control_acc-k*pedal;
    }
    else if (pedal >=0.5)
    {
      double k=3;
      control_acc=control_acc-k*pedal;
    }
  }
  else if(Leader_Acc_pedal>0)
  {
    double pedal=Leader_Acc_pedal/100;
    if(pedal>0.1) 
    { 
      double k=3*(0.4+0.4*pedal);
      control_acc=control_acc+k*pedal;
    }
  }

  // control_acc = 0.05 * (long_distance - Desired_distance);

  // Saturation
  // 踏板限值
  if (control_acc > ACC_LIMIT)
  {
    control_acc = ACC_LIMIT;  // acc limit
  } 
  else if (control_acc < DEACC_LIMIT)
  {
    control_acc = DEACC_LIMIT;  // deacc limit
  }
    
  // cout << "Control_acceleration = "<< to_string(control_acc) << endl;
  return control_acc;
}

/************************************************
 名称：
 功能：
 输入：
 输出：
 ************************************************/
int Control::FindLookAheadPoint(float LookAheadDis, const vector<TrajInfo> msg1){
  int index_max = 0; //The possible max index of LA point
    
  while (msg1[index_max].rel_x < LookAheadDis )
  {
    index_max++;
  }

  int index_la = index_max; //The index of LA point
  
  
  while (msg1[index_la].rel_x*msg1[index_la].rel_x+msg1[index_la].rel_y*msg1[index_la].rel_x-
          LookAheadDis*LookAheadDis > 0)
  {
    index_la--;
  }
  return(index_la);
}
/******************************************************************
 * Function: FindLookAheadPoint;
 * Description: Find the index of the point nearest of the LA point,
 * and return the index;
 * Input: 
 *      float LookAheadDis: Lookahead Distance;
 *      const std::shared_ptr<TrajInfo>& msg1: the pointer to the 
 *                    trajectory infomation;
 * Output:
 *      int index_la: the index of the la point;
 *****************************************************************/

int Control::FindLookAheadPointBezier(float LookAheadDis){
    float DisSum=0;
    int i;
    for(i=1;i<251;i++){
        float dis=sqrt( (BezierX[i]-BezierX[i-1])*(BezierX[i]-BezierX[i-1]) +
                (BezierY[i]-BezierY[i-1])*(BezierY[i]-BezierY[i-1]) );
        DisSum+=dis;
        if(DisSum>LookAheadDis) break;
    }
    return i;
}

/************************************************
 名称：
 功能：
 输入：
 输出：
 ************************************************/
int Control::FindBezierPointID(float LookAheadDis)
{
  float DisSum=0;
  int index=0;

  // 搜索设定距离路点
  for(int i=1;i<250;i++)
  {
    DisSum+=sqrt(pow((BezierX[i]-BezierX[i-1]),2)+pow((BezierY[i]-BezierY[i-1]),2));
    if((DisSum>LookAheadDis)/*|(i>249)*/)
    {
      index=i;
      break;
    }
    if(i>248)
    {
        index=249;
        break;
    }
  }
  
  // 返回路点ID
  return index;
}

/************************************************
 名称：
 功能：       
 输入：
 输出：
 ************************************************/
PID::PID(const float p, const float i, const float d) {
  kp = p;
  ki = i;
  kd = d;
  target = 0;
  actual = 0;
  error_pre = 0; 
  integral=0;        // edit justin
  error=0;           // edit justin
}
/************************************************
 名称：
 功能：
 输入：
 输出：
 ************************************************/
PID::~PID() {}
/************************************************
 名称：
 功能：
 输入：
 输出：
 ************************************************/
float PID::pid_control(float tar, float act) {
  float u;
  target = tar;
  actual = act;
  error = target - act;
  integral += error;
  u = kp * error + ki * integral + kd * (error - error_pre);
  error_pre = error;
  return u;
}

