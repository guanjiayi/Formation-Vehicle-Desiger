#include <iostream>
#include <memory>
#include "../main/FV_Planner.hpp"


using namespace std;

// 规划的路点数

#define SampleTime 0.02
#define PointDistance 0.2
#define ErrorDistance 10

const float Pi = atan(1) * 4;


class Planning
{
    public:    
      Planning();
      ~Planning();
      
      void PathPlanning();
      void SpeedPlanning();        //
      void UpdatePlanning();
    
      // 初始化参考路径
      bool Init(float rel_x, float rel_y, float rel_yam);      // 参考路径初始化
      // 返回初始化参数值
      bool isinit();
      // 路径更新
      void traj_seq_update(float rel_x, float rel_y, float vx_1, float yaw_rate_1,
                       float last_vx_2, float last_yaw_rate_2, float delta_f_2,
                       float rel_yaw);
      // 跟新路径 
      // void TrajUpdate(const std::shared_ptr<ChassisDetail>& msg0);
      void TrajUpdate(const ChassisDetail msg0);
            
      // 存储参考轨迹
      TrajSeq traj;   

      // 存储前车路径
      void SaveTraj();     

    private: 
      bool isInit;      // 路径加载初始化标志位

};
