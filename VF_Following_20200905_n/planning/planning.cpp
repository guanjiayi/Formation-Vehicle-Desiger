#include "planning.hpp"

/**********************************
 
 *********************************/
Planning::Planning()
{
  

}

Planning::~Planning()
{
    
}

/****************************************
funtion:
description:
input:
output:
***************************************/
void Planning::PathPlanning()
{
    cout<<"PathPlanning Success"<<endl;
}

/***************************************
 函数：UpdatePlanning
 功能：路径规划线程主函数
 输入：
 输出：
 ***************************************/
void Planning::UpdatePlanning()
{
    // 变量交互与数据预处理
    ChassisDetail ChassisData;     // 位姿信息
    
    usleep(SAMPLE_TIME);

    isInit=false; 

    // 函数循环主体
    while(1)
    {
      PlanningLock.lock();
      // 领航车信息
      ChassisData.leader_speed=(double)(Leader_Speed)*0.1;                        // 引导车车速 m/s
      ChassisData.leader_acc=(double)Leader_Actual_acc*0.1-15;                    // 引导车加速度 m/s^2
      //ChassisData.leader_brake_pedal=(double)Leader_Brake_pedal_position*0.01;  // 引导车制动踏板开度 %
      //ChassisData.leader_acc_pedal=(double)Leader_ACC_pedal_position*0.4;       // 引导车加速踏板开度 %

      ChassisData.leader_brake_pedal=(double)(Leader_Brake_pedal)*0.01;           // 引导车制动踏板开度 %
      ChassisData.leader_acc_pedal=(double)(Leader_Acc_pedal)*0.01;               // 引导车加速踏板开度 %
      // 
      //ChassisData.steer_angle=(double)Leader_Steering_wheel_angle*0.1-3276.7;     // 引导车方向盘转角
      ChassisData.steer_angle=(double)Follower_steer_angle*0.1;
      ChassisData.x_speed=(double)Follower_Speed*0.1;                             // 跟随车车速 m/s
      // ChassisData.x_speed=1.2;
      ChassisData.x_acc=(double)(Follower_La_acc)*0.1-15.0;                       // 跟随车纵向加速度 m/s^2
      ChassisData.pedal_acc=0;                                                    // 跟随车加速踏板开度%
      ChassisData.pedal_brake=0;                                                  // 跟随车制动踏板开度%
      ChassisData.uwb_attitude=(double)UWB_zitai;                                 // deg
      ChassisData.uwb_azimuth=(double)UWB_fangwei;                                // deg
      ChassisData.uwb_distance=(double)(UWB_distance)/100;                        // m
      ChassisData.uwb_status=Leader_Check;                                        // 
      ChassisData.follower_yaw_rate=Follower_yaw_rate;                            //deg/s

      PlanningLock.unlock();
      
      // 路径转换
      TrajUpdate(ChassisData);

      PlanningThflag=false;
      // 存储路径至
      SaveTraj();
      
      PlanningThflag=true;

      usleep(SAMPLE_TIME);
    }    
         
}

/****************************************************
 名称：
 功能：
 输入：
 输出：
 ****************************************************/

// void Planning::TrajUpdate(const std::shared_ptr<ChassisDetail>& msg0) {
void Planning::TrajUpdate(const ChassisDetail msg0) {
  static float last_vx_2;
  static float vx_2;
  static float last_yaw_rate_2;
  static float yaw_rate_2;

  float distance = msg0.uwb_distance;
  float azimuth_angle = msg0.uwb_azimuth;
  float lat_distance = -distance * sin(azimuth_angle / 180 * M_PI);      // 横向距离，指向车辆行驶方向左侧 
  float long_distance = distance * cos(azimuth_angle / 180 * M_PI);      // 纵向距离，沿着车辆行驶方向，车辆前向行驶方向为正
  float vx_1 = msg0.leader_speed;                                        // 领航车车速

  float delta_f_2 = -(msg0.steer_angle - 4.8505) / 24.1066;              // 估算引导车航向
  float yaw_rate_1 = 0;
  float rel_yaw = 0;  // todo:add leader yaw rate in algorithm
  
  // cout<<"isInit:"<<isInit<<endl;
  if (!isinit()) {
    //cout<<"Inited:"<<endl;
    if( distance > 1 ){
        last_vx_2 = 0;
        last_yaw_rate_2 = 0;
        vx_2 = msg0.x_speed;
        yaw_rate_2 = msg0.follower_yaw_rate;        
        Init(long_distance, lat_distance, rel_yaw);
        cout <<"Planner Inited, distance:"<< distance;
    }
    else{
      return;
    }    
  } 
  else {
    last_vx_2 = vx_2;                     // 上一帧跟随车车速
    vx_2 = msg0.x_speed;                  // 跟随车车速
    last_yaw_rate_2 = yaw_rate_2;         // 上一帧跟随车横摆角速度
    yaw_rate_2=msg0.follower_yaw_rate;    // 跟随车横摆角速度
    traj_seq_update(long_distance, lat_distance, vx_1, yaw_rate_1,
                            last_vx_2, last_yaw_rate_2, delta_f_2, rel_yaw);
  }
  return;
}

/****************************************************
 名称：
 功能：
 输入：
 输出：
 ****************************************************/

bool Planning::Init(float rel_x, float rel_y, float rel_yaw) {
  
  for (int i = 0; i < 5; i++) 
  traj.Seqv[i].clear();

  for (int i = 0; i < PointsNumber; i++) {
    traj.Seqv[0].push_back(rel_x / (PointsNumber - 1) * i);
    traj.Seqv[1].push_back(rel_y / (PointsNumber - 1) * i);
    traj.Seqv[2].push_back(rel_yaw);  //相对转角
    traj.Seqv[3].push_back(3);
    traj.Seqv[4].push_back(0);
  }
  traj.number = PointsNumber;
  cout <<"traj_frist:"<<traj.number<<endl;
  return isInit = true;
   
}

/****************************************************
 名称：
 功能：
 输入：
 输出：
 ****************************************************/
bool Planning::isinit() { 
  return isInit; 
  };

/*************************************************
 名称：traj_seq_update
 功能：更新路径
 输入：
 输出：
 *************************************************/
void Planning::traj_seq_update(float rel_x, float rel_y, float vx_1,
                              float yaw_rate_1, float last_vx_2,
                              float last_yaw_rate_2, float delta_f_2,
                              float rel_yaw) {
  // delta_f_2表示后车前轮转角
  // 参数定义
  float lf = 1.1, lr = 2.8;  //质心到前后轴距离
  float delta_yaw;           //跟随车转动角度
  TrajSeq traj_seq;
  TrajSeq &last_traj_seq = traj;

  // 第一次规划采用的参数
  if (!isinit()) {
    return;
  } 
  else {
    delta_yaw = last_yaw_rate_2 * SampleTime / 180 * Pi;                  // Sampletime里转动的角度
    float x_0 = last_vx_2 * SampleTime;                                   // Sampletime里x方向移动的距离
    float y_0 = x_0 * lr / (lf + lr) * tan(delta_f_2 / 180 * Pi);         // Sampletime里y方向移动的距离

    traj_seq = last_traj_seq;

    for (int i = 0; i < traj_seq.number; i++) {
      traj_seq.Seqv[0][i] = traj_seq.Seqv[0][i] - x_0;
      traj_seq.Seqv[1][i] = traj_seq.Seqv[1][i] - y_0;
      float tmp1 = cos(delta_yaw) * traj_seq.Seqv[0][i] + sin(delta_yaw) * traj_seq.Seqv[1][i];
      float tmp2 = -sin(delta_yaw) * traj_seq.Seqv[0][i] + cos(delta_yaw) * traj_seq.Seqv[1][i];       //使用坐标转换矩阵
      traj_seq.Seqv[0][i] = tmp1;
      traj_seq.Seqv[1][i] = tmp2;
      traj_seq.Seqv[2][i] = traj_seq.Seqv[2][i] - SampleTime * last_yaw_rate_2;      //更新yaw角
    }

    //剔除车后的轨迹点
    for (vector<float>::iterator it = traj_seq.Seqv[0].begin();
         it != traj_seq.Seqv[0].end();) {
      if (*it < 0) {
        traj_seq.number--;
        it = traj_seq.Seqv[0].erase(it);
        traj_seq.Seqv[1].erase(traj_seq.Seqv[1].begin());
        traj_seq.Seqv[2].erase(traj_seq.Seqv[2].begin());
        traj_seq.Seqv[3].erase(traj_seq.Seqv[3].begin());
        traj_seq.Seqv[4].erase(traj_seq.Seqv[4].begin());
      } 
      else {
        break;
      }
    }

    //判断新点的距离
    float old_x = 0, old_y = 0;  //若没有点则默认为本车点
    if (traj_seq.number > 0) {
      old_x = traj_seq.Seqv[0][traj_seq.number - 1];
      old_y = traj_seq.Seqv[1][traj_seq.number - 1];
    }
    float dis = sqrt((old_x - rel_x) * (old_x - rel_x) +
                     (old_y - rel_y) * (old_y - rel_y));

    // to far then reset;
    if(dis > ErrorDistance){
        //Init(rel_x,rel_y,rel_yaw);
        traj = traj_seq;
        return;
    }
    //加入新轨迹点
    if (dis > PointDistance) {
      traj_seq.Seqv[0].push_back(rel_x);
      traj_seq.Seqv[1].push_back(rel_y);
      traj_seq.Seqv[2].push_back(rel_yaw);
      traj_seq.Seqv[3].push_back(vx_1);
      traj_seq.Seqv[4].push_back(yaw_rate_1);
      traj_seq.number++;
    }
  }
  traj = traj_seq;
}
/****************************************************
 名称：SaveTraj
 功能：保存前导车路径
 输入：
 输出：
 ****************************************************/

void Planning::SaveTraj()
{
   if(isinit()){

      // 清除轨迹点坐标信息
      //LeaderTraj.clear();  
      vector<TrajInfo>trajpoint;
      trajpoint.clear();
      TrajInfo trajinfo;    
      // 转存前车路径点  
      for (int i = 0; i < traj.Seqv[0].size(); i++) {
        trajinfo.rel_x=traj.Seqv[0][i];
        trajinfo.rel_y=traj.Seqv[1][i];
        trajpoint.push_back(trajinfo);        // 将路径压入路径向量         
      } 
      
      // cout<<traj.Seqv[0].size()<<endl;

      // 清除前车轨迹点并更新存储轨迹点      
      PlanningLock.lock();
      LeaderTraj.clear();      
      //for(int)
      //cout<<trajpoint[199].rel_x<<endl;      
      LeaderTraj.assign(trajpoint.begin(),trajpoint.end());
      //cout<<LeaderTraj[199].rel_x<<endl; 
      PlanningLock.unlock();         
  }  
  
}

/****************************************************
 名称：
 功能：
 输入：
 输出：
 ****************************************************/
TrajSeq::TrajSeq() {
  number = 0;
  for (int i = 0; i < 5; i++) Seqv[i].clear();
}

/****************************************************
 名称：
 功能：
 输入：
 输出：
 ****************************************************/
void TrajSeq::Print() {
  for (int i = 0; i < number; i++) {
    
    cout << "point " << i << " :";
    for (int j = 0; j < 5; j++) 
    cout << Seqv[j][i] << " ";
    cout << endl;

  }
  cout << endl;
}


