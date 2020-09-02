#include "canmessage.hpp"
#include "../decision/decision.hpp"
#include "../main/FV_Planner.hpp"


CANMessage::CANMessage()
{

}

CANMessage::~CANMessage()
{
    
}

void CANMessage::GetCANMessage()
{
    cout<<"GetCANMessage Success"<<endl;
}

/***********************************************
 名称：CAN0_update
 功能：发送控制数据线程函数
 输入：
 输出：
 ***********************************************/
void CANMessage::CAN0_update()
{
    for(;;)
        switch(State)
            {

            case DRIVER_MODE:
            {
                usleep(SAMPLE_TIME);
                break;
            }

            case READY_STATE:
            {
                control_mut.lock();
                CANMessage::CAN_send(Con2CAN_steer(1,33000,100),
                                        CONTROL_STEER_MSG);
                CANMessage::CAN_send(Con2CAN_acc(0,100,0),
                                        CONTROL_ACC_MSG);
                control_mut.unlock();
                usleep(SAMPLE_TIME);
                //cout << "[READY STATE]CAN0 data is updating..." << endl;
                break;
            }

            case RUN_STATE:
            {
                control_mut.lock();

                if((Control_steer_angle > 32767-5000) && (Control_steer_angle < 32767+5000)){
                    //cout<<"angle";
                    CANMessage::CAN_send(Con2CAN_steer(1,Control_steer_angle,Control_steer_velocity),
                                            CONTROL_STEER_MSG);
                }

                if(Control_acceleration > 50 && Control_acceleration < 250){
                    //cout<<"acc";
                    //cout << "lll " << Control_acceleration << endl;
                    CANMessage::CAN_send(Con2CAN_acc(Control_mode,Control_acceleration,Control_pressure),
                                            CONTROL_ACC_MSG);
                }
                //cout<<
                control_mut.unlock();
                usleep(SAMPLE_TIME);
                //cout << "[RUN STATE]CAN0 data is updating..." << endl;
                break;
            }
            case FINISH_STATE:
            {
                control_mut.lock();
                CANMessage::CAN_send(Con2CAN_steer(0,32767,100),
                                        CONTROL_STEER_MSG);
                CANMessage::CAN_send(Con2CAN_acc(1,100,0),
                                        CONTROL_ACC_MSG);
                control_mut.lock();
                usleep(SAMPLE_TIME);
                //cout << "[RUN STATE]CAN0 data is updating..." << endl;
                break;
                
            }
            default:
            {
                control_mut.lock();
                CANMessage::CAN_send(Con2CAN_steer(0,Control_steer_angle,Control_steer_velocity),
                                        CONTROL_STEER_MSG);
                CANMessage::CAN_send(Con2CAN_acc(1,130,0),
                                        CONTROL_ACC_MSG);
                control_mut.unlock();
                usleep(SAMPLE_TIME);
                //Test mode
                //cout << "[DRIVER MODE]CAN0 data is not updating..." << endl;
            }
            

        }
            //cout << "CAN0 data is updating..." << endl;

}
//Send all messages

/***************************************************
 名称：CAN_receive
 功能：接受CAN消息
 输入：
 id 消息id
 mes_length 消息长度
 EFF
 CAN_channel CAN通道
 输出：
 ***************************************************/
void CANMessage::CAN_receive(int id,int msg_length,bool EFF,int CAN_channel){
    for(;;)
    {
        // 读取ID为ACC_ID的CAN消息 
        if(id == VEHICLE_ACC_ID)
        {
            static int CAN_msg[9] = {0};
            
            CAN_get_msg(id,EFF,CAN_channel,CAN_msg);

            if(CAN_msg[8]==1)
            {
                continue;
            }
            else
            {
                CAN2Val_acc(CAN_msg);
                CAN2Val_speed(CAN_msg);
            }
            usleep(SAMPLE_TIME);
            // sleep(Sample_Time);            
        }
        
        // 读取ID为POSTION_ID的CAN消息
        else if(id == UWB_POSITION_ID)
        {
            static int CAN_msg[9] = {0};
            
            CAN_get_msg(id,EFF,CAN_channel,CAN_msg);
            if(CAN_msg[8]==1)
            {
                continue;
            }
            else
            {
                CAN2Val_UWB_position(CAN_msg);
            }
            usleep(SAMPLE_TIME);
        }

        // 读取ID为STATE_ID的CAN消息 
        else if(id == UWB_LEADERSTATE_ID)
        {
            // static int CAN_msg[9] = {0};
            static int CAN_msg[9] = {0};
            CAN2Val_UWB_leaderstate(CAN_get_msg(id,EFF,CAN_channel,CAN_msg));
            //usleep(SAMPLE_TIME/16);
        }
        else if(id == VEHICLE_RATE_ID)
        {
            static int CAN_msg[9] = {0};
            CAN_get_msg(id,EFF,CAN_channel,CAN_msg);
            if(CAN_msg[8]==1)
            {
                continue;
            }
            else
            {
                CAN2Val_Yaw_Rate(CAN_msg);
            
            }
            
            // CAN2Val_Yaw_Rate(CAN_get_msg(id,EFF,CAN_channel,CAN_msg));
            usleep(SAMPLE_TIME);

        }
        
    }
}

//Receive a certain ID CANmsg
/**********************************************
 名称：
 功能：
 输入：
 输出：
 **********************************************/
void CANMessage::CAN_send(int *message_ptr,int id,int msg_length,bool EFF, int CAN_channel){
    //cout << "Sending..." << endl;

    /***********************Sockek_CAN config*****************************/
    //CAN send configuration
    int socket_word,nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame[2] = {{0}};
    socket_word = socket(PF_CAN,SOCK_RAW,CAN_RAW);

    if(CAN_channel == 0)
        strcpy(ifr.ifr_name,"can0");
    else if(CAN_channel == 1)
        strcpy(ifr.ifr_name,"can1");

    ioctl(socket_word,SIOCGIFINDEX,&ifr);
    addr.can_family =  AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    bind(socket_word,(struct sockaddr *)&addr,sizeof(addr));
    setsockopt(socket_word,SOL_CAN_RAW,CAN_RAW_FILTER,NULL,0);
    /***********************************************************************/

    //CAN Information
    if(EFF)
        frame[0].can_id = CAN_EFF_FLAG | id;
    else
        frame[0].can_id = id;//CAN ID NOTICE:the input should map to HEX
    frame[0].can_dlc = msg_length;//Message Length

    //CAN message
    for(int i=0;i<msg_length;i++){
        frame[0].data[i] = *(message_ptr+i);
        //printf("%X",frame[0].data[i]);
    }
    //cout<<endl;


    //Send CAN message
    nbytes = write(socket_word,&frame[0],sizeof(frame[0]));
    // Print CAN message or error
    if(nbytes != sizeof(frame[0])){
        if(CAN_SEND_CHECK|Show_switch)
            cout << "CAN Send Error! Check bitrate. " << endl;
    }
    else{
        if(CAN_SEND_CHECK|Show_switch){
            cout << "Sending: ";
            if(id == CONTROL_STEER_ID)
                cout << "(control steer)";
            else if(id == CONTROL_ACC_ID)
                cout << "(control acc)";
            cout << " CAN ID 0x" << hex << id << " : ";
            for(int i=0; i< msg_length; i++) {
                //CANmsg_acc[i] = *(CANmsg_acc_ptr + i);
                cout << hex << message_ptr[i] << " ";
            }
            cout << endl;
        }
    };
    close(socket_word);
}
//Send a certain ID CANmsg

/****************************************************************************
 名称：CAN_Get_msg 
 功能：建立链接读取CAN消息
 输入: 
 id 消息id
 EFF
 CAN_channel CAN通道
 输出：
 CAN_msg CAN消息  
 ****************************************************************************/
int * CANMessage::CAN_get_msg(int id, bool EFF, int CAN_channel,int *CAN_msg)
{
    //cout << "Receiving ID " << id << " ..." << endl;
    /***********************Sockek_CAN config*****************************/
    int socket_word, nbytes;
    struct sockaddr_can addr;
    struct ifreq ifr;
    struct can_frame frame;
    struct can_filter rfliter[1];
    socket_word = socket(PF_CAN, SOCK_RAW, CAN_RAW);

    //Choose CAN channel
    if(CAN_channel == 0)
        strcpy(ifr.ifr_name, "can0" );
    else if(CAN_channel == 1)
        strcpy(ifr.ifr_name, "can1");    

    ioctl(socket_word, SIOCGIFINDEX, &ifr);
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    struct can_filter rfilter[1];
    bind(socket_word, (struct sockaddr *)&addr, sizeof(addr));

    //定义接收规则，只接收表示符等于 id 的报文
    rfilter[0].can_id = id;
    if(EFF)
        rfilter[0].can_mask = CAN_EFF_MASK;
    else
        rfilter[0].can_mask = CAN_SFF_MASK;
    //设置过滤规则
    setsockopt(socket_word, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter));

    nbytes = read(socket_word, &frame, sizeof(frame));
    if(nbytes > 0)
    {
        //cout << "Receiving..." << endl;
    }
    /**********************************************************************/
    //cout << "**********************" << endl;
    // Convert CANmsg to int ptr
    if(CAN_RECEIVE_CHECK|Show_switch){
        cout << "Receiving: ";
        if(id == VEHICLE_ACC_ID)
            cout << "(vehicle acc)";
        //else if(id == VEHICLE_SPEED_ID)
        //    cout << "(vehicle speed)";
        else if(id == UWB_POSITION_ID)
            cout << "(UWB position)";
        else if(id == UWB_LEADERSTATE_ID)
            cout << "(UWB leader state)";
        cout << " CAN ID 0x" << hex << id << ": ";
    }
    msg_mut.lock();
    //static int CAN_msg[8] = {0};
    CAN_msg[8]=1;
    if( (id%65536) == (frame.can_id%65536))
    {
        CAN_msg[8]=0;
    }
    for (int i=0;i<8;i++)
    {
        CAN_msg[i] = (int)frame.data[i];
        if(CAN_RECEIVE_CHECK|Show_switch)
            cout << CAN_msg[i] << " " ;
    }
    if(CAN_RECEIVE_CHECK|Show_switch)
        cout << endl;

    close(socket_word);
    msg_mut.unlock();
    return CAN_msg;
    //TODO:necessay to set socket every time?
}
//get CAN message

/****************************************************************************
 名称：Con2CAN_steer
 功能：将方向盘转角信息转化为CAN信号
 输入：
 输出：
msg_steer 方向盘转角CAN信号
 ****************************************************************************/
int * CANMessage::Con2CAN_steer(int steer_enable,int steer_angle,int steer_velocity){
    static int msg_steer[8] = {0};
    msg_steer[0] = steer_enable;
    msg_steer[1] = steer_velocity/4;
    msg_steer[2] = steer_angle%256;
    msg_steer[3] = steer_angle/256;
    return msg_steer;
}
//Convert steer information to CANmsg
/****************************************************************************
 名称：Con2CAN_acc
 功能：将加速信息转化为CAN信号
 输入：
 输出：
msg_acc 加速控制CAN信号
 ****************************************************************************/
int * CANMessage::Con2CAN_acc(int control_mode,int acc_value, int pressure_value){
    static int msg_acc[8] = {0};
    static int loop_number = 0;
    
    if(control_mode == 0)
    {

    }
    else
    {
        if(acc_value > 150)
        {
            control_mode = 2;
        }
        else if(acc_value < 150)
        {
             control_mode = 1;
        }
        else control_mode = 0;
    }
    msg_acc[2] = control_mode * 16;
    
    // edit justin 20200825
    //cout << "acc value = " << acc_value << endl;
    //cout << "Control Mode = " << control_mode << endl;

    if(control_mode==0)
    {//mode 0:No Brake
        msg_acc[0] = 150;
        msg_acc[1] = 0;
    }
    else if(control_mode==2){       //mode 1:Require deacc
        msg_acc[0] = acc_value%256;
        msg_acc[1] = acc_value/256;
    }
    else if(control_mode==1){      //mode 2:Require acc
        msg_acc[0] = acc_value%256;
        msg_acc[1] = acc_value/256;
    }
    msg_acc[7] = loop_number%16;
    loop_number ++;
    return msg_acc;
}
//Convert acc information to CANmsg

/*********************************************
 名称：CAN2Val_acc
 功能：解析CAN加速度信息
 输入：
 输出：
 Follower_La_acc 跟随车加速度信息
 *********************************************/
void CANMessage::CAN2Val_acc(int *CANmsg_acc)
{
    int origin_follower_acc;    
    origin_follower_acc = CANmsg_acc[1] + CANmsg_acc[2] * 256;   //[0,300]

    if(origin_follower_acc>50 && origin_follower_acc<250 && 
    origin_follower_acc-Follower_La_acc<80 && origin_follower_acc-Follower_La_acc>-80)
        Follower_La_acc = origin_follower_acc;
}

//Convert CANmsg to follower acc value
/***********************************************************
 名称：CAN2Val_UWB_posotion
 功能：解析UWB姿态和方位CAN信号
 输入：
 CANmsg_UWB UWB的CAN消息
 输出：
 UWB_distance UWB 距离
 UWB_fangwei  UWB 方位
 UWB_zitai    UWB 姿态
 **********************************************************/
void CANMessage::CAN2Val_UWB_position(int*CANmsg_UWB){
    
    // edit justin 20200831
    mutex mut;
    lock_guard<mutex> lock(mut);      

    UWB_distance = 256 * CANmsg_UWB[3] + CANmsg_UWB[2];
    //cout<<"dis="<<UWB_distance<<endl;

    if (CANmsg_UWB[5] >= 0x80)
    {
        UWB_fangwei = 256 * CANmsg_UWB[5] + CANmsg_UWB[4] - 65536;
    }
    else {
        UWB_fangwei = 256 * CANmsg_UWB[5] + CANmsg_UWB[4];
    }
    //cout<<"fangwei="<<UWB_fangwei<<endl;
    if (CANmsg_UWB[7] >= 0x80)
    {
        UWB_zitai = 256 * CANmsg_UWB[7] + CANmsg_UWB[6] - 65536;
    }
    else {
        UWB_zitai = 256 * CANmsg_UWB[7] + CANmsg_UWB[6];
    }
}
//Convrt CANmsg to UWB position value
/**************************************************
 名称:CAN2Val_UWB_leaderstate
 功能：解析前车CAN状态数据
 输入：
 msg CAN消息
 输出： 
 **************************************************/
void CANMessage::CAN2Val_UWB_leaderstate(int*msg){
    if(msg[0] == 0xA1)
        CAN2Val_acc_pedal(msg);
    else if(msg[0] == 0xA2)
        CAN2Val_brake(msg);
    else if(msg[0] == 0xA3)
        CAN2Val_steering_wheel(msg);
    else if(msg[0] == 0xA4)
        CAN2Val_wheel(msg);
    else if(msg[0] == 0xA5)
        CAN2Val_la_yr(msg);
    else if(msg[0] == 0xA6)
        CAN2Val_gear_position(msg);
    else if(msg[0] == 0xA7)
        CAN2Val_pedal_angle(msg);
    //CANmsg_UWB_state[8]

}
//Convert CANmsg to leader state value
/***********************************************
 名称：CAN2Val_speed
 功能：解析跟随车车速
 输入：
 输出：
 Follower_Speed 跟随车车速
 ***********************************************/
void CANMessage::CAN2Val_speed(int*CANmsg_speed){
    int origin_follower_speed;
    origin_follower_speed = CANmsg_speed[3] + CANmsg_speed[4] * 256;//[0,800]
    if(origin_follower_speed<200 && origin_follower_speed-Follower_Speed<80 && origin_follower_speed-Follower_Speed>-80)
        Follower_Speed = origin_follower_speed;
}

/***********************************************
 名称：CAN2Val_acc_pedal
 功能：解析引导车加速踏板开度信号
 输入：
 输出：
 
 ***********************************************/
void CANMessage::CAN2Val_acc_pedal(int*CANmsg_acc_pedal){
    mutex mut;
    lock_guard<mutex> lock(mut);
    Leader_ACC_pedal_position = CANmsg_acc_pedal[1];
    Leader_Remote_position = CANmsg_acc_pedal[4];
    //cout << "Leader_ACC_pedal_position = " << Leader_ACC_pedal_position << endl;
}

/***********************************************
 名称：CAN2Val_brake
 功能：解析引导车制动踏板信号
 输入：
 输出：
 
 ***********************************************/
void CANMessage::CAN2Val_brake(int*CANmsg_brake){
    mutex mut;
    lock_guard<mutex> lock(mut);
    Leader_Brake_pedal_position = CANmsg_brake[1];
    int origin_leader_acc;
    origin_leader_acc = CANmsg_brake[2] + CANmsg_brake[3] * 256;//[0,300]
    //cout << "origin_leader_acc = " << origin_leader_acc << endl;
    if(origin_leader_acc>50 && origin_leader_acc<250 && origin_leader_acc-Leader_Actual_acc<80 && origin_leader_acc-Leader_Actual_acc>-80){
        Leader_Actual_acc = origin_leader_acc;
        //cout << "Yes " << Leader_Actual_acc << endl; 
    }

    int origin_leader_speed;
    origin_leader_speed = CANmsg_brake[4] + CANmsg_brake[5] * 256;//[0,800]
    if(origin_leader_speed<200 && origin_leader_speed-Leader_Speed<80 && origin_leader_speed-Leader_Speed>-80)
        Leader_Speed = origin_leader_speed;

    Leader_Pressure = CANmsg_brake[6];
    //cout << "Leader_Speed = " << Leader_Speed << endl;
}

/***********************************************
 名称：CAN2Val_steering_wheel
 功能：解析领航车方向盘转角
 输入：
 输出： 
 ***********************************************/
void CANMessage::CAN2Val_steering_wheel(int*CANmsg_steering_wheel){
    mutex mut;
    lock_guard<mutex> lock(mut);
    if(CANmsg_steering_wheel[2] >= 0x80)
        Leader_Steering_wheel_angle = (CANmsg_steering_wheel[2] * 256 + CANmsg_steering_wheel[1]) - 0xFFFF - 1;
    else
        Leader_Steering_wheel_angle = (CANmsg_steering_wheel[2] * 256 + CANmsg_steering_wheel[1]);
    Leader_Steering_wheel_speed = CANmsg_steering_wheel[3];
    Leader_Steering_wheel_state = CANmsg_steering_wheel[4];
    Leader_Count = CANmsg_steering_wheel[5];
    Leader_Check = CANmsg_steering_wheel[7];
}

/***********************************************
 名称：CAN2Val_wheel
 功能：解析领航车车速
 输入：
 输出：
 
 ***********************************************/
void CANMessage::CAN2Val_wheel(int*CANmsg_wheel_speed){
    mutex mut;
    lock_guard<mutex> lock(mut);
    Leader_Wheel_speed = CANmsg_wheel_speed[3] * (256 * 256) + CANmsg_wheel_speed[2] * 256 + CANmsg_wheel_speed[1];
}

/***********************************************
 名称：CAN2Val_la_yr
 功能：解析领航车侧向加速度
 输入：
 输出： 
 ***********************************************/
void CANMessage::CAN2Val_la_yr(int*CANmsg_la_yr){
    mutex mut;
    lock_guard<mutex> lock(mut);
    Leader_La_acc = CANmsg_la_yr[1] + CANmsg_la_yr[2] * 256;
    Leader_Yr_speed = CANmsg_la_yr[1] + CANmsg_la_yr[2] * 256;
}

/***********************************************
 名称：CAN2Val_gear_position
 功能：解析领航车档位
 输入：
 输出： 
 ***********************************************/
void CANMessage::CAN2Val_gear_position(int*CANmsg_gear_position){
    mutex mut;
    lock_guard<mutex> lock(mut);
    Leader_Target_gear = CANmsg_gear_position[1];
    Leader_Current_gear = CANmsg_gear_position[5];
}
/***********************************************
 名称：CAN2Val_pedal_angle
 功能：解析领航车开度信号
 输入：
 输出：
 
 ***********************************************/
void CANMessage::CAN2Val_pedal_angle(int*CANmsg_pedal_angle){
    mutex mut;
    lock_guard<mutex> lock(mut);
    Leader_Acc_pedal = CANmsg_pedal_angle[3] + CANmsg_pedal_angle[4] * 256;
    Leader_Brake_pedal = CANmsg_pedal_angle[1] + CANmsg_pedal_angle[2] * 256;
}

//Convert CANmsg to follower speed
/************************************************************************************************/

/*****************************************************
 名称：
 描述：
 输入：
 输出：
 *****************************************************/
void CANMessage::CAN2Val_Yaw_Rate(int*CANMsg_Yaw_Rate)
{
    mutex mut;
    lock_guard<mutex>lock(mut);
    if (CANMsg_Yaw_Rate[5] >= 0x80)
    {
        Follower_yaw_rate = (256 * CANMsg_Yaw_Rate[3] + CANMsg_Yaw_Rate[2] - 65536)/131;
    }
    else
    {
        Follower_yaw_rate = ( 256 * CANMsg_Yaw_Rate[3] + CANMsg_Yaw_Rate[2])/131;
    }
        
    // Follower_yaw_rate=(CANMsg_Yaw_Rate[2]+CANMsg_Yaw_Rate[3]*256)/131;   
}