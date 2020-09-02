#ifndef LEADERFOLLOWING_COMMUNICATION_H
#define LEADERFOLLOWING_COMMUNICATION_H

#endif //LEADERFOLLOWING_COMMUNICATION_H

#define CAN_EFF_FLAG 0x80000000U //Extended Frame Mark
#define CAN_EFF_MASK 0x1FFFFFFFU //Extended Frame format

//CAN receive
// MSG=(ID,dlc,EFF,CAN_channel)
#define UWB_POSITION_ID 0x650
#define UWB_POSITION_MSG UWB_POSITION_ID,8,1,0
#define UWB_LEADERSTATE_ID 0x03100000

#define UWB_LEADERSTATE_MSG UWB_LEADERSTATE_ID,8,1,0
//#define VEHICLE_SPEED_ID 0x0cfe6c03
//#define VEHICLE_SPEED_MSG VEHICLE_SPEED_ID,8,1,1
#define VEHICLE_ACC_ID 0x18f02501
#define VEHICLE_ACC_MSG VEHICLE_ACC_ID,8,1,0

#define VEHICLE_RATE_ID 0x05A                       // 后车横摆角速度报文ID
#define VEHICLE_RATE_MSG VEHICLE_RATE_ID,8,1,0      

//CAN send
#define CONTROL_STEER_ID 0x4ef8480
#define CONTROL_STEER_MSG CONTROL_STEER_ID,4,1,0
#define CONTROL_ACC_ID 0xc040b2a
#define CONTROL_ACC_MSG CONTROL_ACC_ID,8,1,0


#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <iostream>
#include <string.h>
#include <string>

using namespace std;

class CANMessage
{
public:
    CANMessage();
    ~CANMessage();
    
    void GetCANMessage();    

    // static void CAN0_update();         //send all signals
    // static void CAN_send(int *message_ptr,int id,int msg_length,bool EFF, int CAN_channel);
    // static void CAN_receive(int id,int msg_length,bool EFF,int CAN_channel);
    // static int * CAN_get_msg(int id,bool EFF,int CAN_channel,int *CAN_msg);

    void CAN0_update();         //send all signals
    void CAN_send(int *message_ptr,int id,int msg_length,bool EFF, int CAN_channel);
    void CAN_receive(int id,int msg_length,bool EFF,int CAN_channel);
    int * CAN_get_msg(int id,bool EFF,int CAN_channel,int *CAN_msg);



private:

    // static int * Con2CAN_steer(int steer_enable,int steer_angle,int steer_velocity);
    // static int * Con2CAN_acc(int control_mode, int acc_value, int pressure_value);

    // //CAN Receive
    // static void CAN2Val_acc(int *message_ptr);
    // static void CAN2Val_UWB_position(int *message_ptr);
    // static void CAN2Val_UWB_leaderstate(int *message_ptr);
    // static void CAN2Val_speed(int *message_ptr);

    // static void CAN2Val_acc_pedal(int *message_ptr);
    // static void CAN2Val_brake(int *message_ptr);
    // static void CAN2Val_steering_wheel(int *message_ptr);
    // static void CAN2Val_wheel(int *message_ptr);
    // static void CAN2Val_la_yr(int *message_ptr);
    // static void CAN2Val_gear_position(int *message_ptr);
    // static void CAN2Val_pedal_angle(int *message_ptr);

    int * Con2CAN_steer(int steer_enable,int steer_angle,int steer_velocity);
    int * Con2CAN_acc(int control_mode, int acc_value, int pressure_value);

    //CAN Receive
    void CAN2Val_acc(int *message_ptr);
    void CAN2Val_UWB_position(int *message_ptr);
    void CAN2Val_UWB_leaderstate(int *message_ptr);
    void CAN2Val_speed(int *message_ptr);

    void CAN2Val_acc_pedal(int *message_ptr);
    void CAN2Val_brake(int *message_ptr);
    void CAN2Val_steering_wheel(int *message_ptr);
    void CAN2Val_wheel(int *message_ptr);
    void CAN2Val_la_yr(int *message_ptr);
    void CAN2Val_gear_position(int *message_ptr);
    void CAN2Val_pedal_angle(int *message_ptr);
    void CAN2Val_Yaw_Rate(int *message_ptr);
};