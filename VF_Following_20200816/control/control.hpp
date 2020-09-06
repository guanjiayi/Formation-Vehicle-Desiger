#define ACC_LIMIT 0.5
#define DEACC_LIMIT -4
#include <iostream>
using namespace std;

class Control{

    public:
        Control();
        ~Control();
        void SpeedControl();
        void SteeringControl();

    private:
        // args
        static constexpr float k_a = 0.1;
        static constexpr float k_v = 0.05;
        static constexpr float k_d = 0.05;

        /*
        static constexpr float k_a1 = 0.2;
        static constexpr float k_v1 = 0.1;
        static constexpr float k_d1 = 0.1;
        */

        // float signal variable
        /*nvidia
        static float leader_speed;
        static float follower_speed;
        static float leader_la_acc;
        static float follower_la_acc;
        static float fangwei_angle;
        static float zitai_angle;
        */

        float leader_speed;
        float follower_speed;
        float leader_la_acc;
        float follower_la_acc;
        float fangwei_angle;
        float zitai_angle;
        
        float leader_acc_pedal_position;
        float leader_remote_position;
        float leader_brake_pedal_position;
        float leader_actual_acc;
        float leader_pressure;
        float leader_steering_wheel_angle;
        float leader_steering_wheel_speed;
        //static float leader_steering_wheel_state;
        //static float leader_count;
        //static float leader_check;
        float leader_wheel_speed;

        float leader_yr_speed;
        //static float leader_target_gear;
        //static float leader_current_gear;
        float leader_acc_pedal;
        float leader_brake_pedal;

        float long_distance;
        float lat_distance;
        float frontwheel_steer_angle;
        float steer_wheel_angle;


        float control_steer;
        float control_acc;
        float control_brake_pressure;
        // vehicle parads
        static constexpr float L = 5.0;
        static constexpr float r = 1.0;

    public:
        //Control();
        //Control(float k_v, float k_d);
        static void Control_update();
        static float Caculate_steer(float lat_distance, float long_distance);
        static float Caculate_acc(float v1, float v2, float a1, float long_distance);

        

};
class PID{
    public:
        PID();
        ~PID(){};
        PID(float p,float i,float d);
        float pid_control(float tar,float act);
        void pid_show();
    private:
        float kp;
        float ki;
        float kd;
        float target;
        float actual;
        float error;
        float error_pre;
        float integral;
};
