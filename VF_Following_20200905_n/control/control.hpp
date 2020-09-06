#define ACC_LIMIT 0.5
#define DEACC_LIMIT -4
#include <iostream>
#include "../main/FV_Planner.hpp"

using namespace std;

class Control{

    public:
        Control();
        ~Control();
       
        void UpdateControl();
        

    private:

     float control_acc = 0;
     float control_steer = 0;
     float lookahead_x = 0;
     float lookahead_y = 0;
     float err_lat = 0;
     float steer_PID_kp,steer_PID_ki,steer_PID_kd;
     float BezierX[251], BezierY[251];

     float Caculate_steer(const ChassisDetail msg0,const vector<TrajInfo> msg1);
     float Caculate_steer_pure(const ChassisDetail msg0);
     float Caculate_acc(const ChassisDetail msg0);
     void DealTraj(const vector<TrajInfo> msg1);
     void BezierFitting(const vector<TrajInfo> msg1);
     float CalBezierLoc(int n, float t, float p[]);
     int FindLookAheadPoint(float LookAheadDis, const vector<TrajInfo> msg1);
     int FindLookAheadPointBezier(float LookAheadDis);   
     int FindBezierPointID(float LookAheadDis);



};


class PID {
 private:
  float kp;
  float ki;
  float kd;
  float target;
  float actual;
  float error;
  float error_pre;
  float integral;

 public:
  PID(const float p, const float i, const float d);
  ~PID();
  float pid_control(float tar, float act);
};
