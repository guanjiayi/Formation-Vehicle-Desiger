#include "decision.hpp"
#include <iostream>
#include <string>
#include <string.h>
#include "../main/FV_Planner.hpp"

using namespace std;

Decision::Decision()
{

}

Decision::~Decision()
{
    
}

void Decision::CurrentState()
{
    std::cout<<"CurrentState Success"<<endl;
}


/*static const char help_string[] =
        "\n"
        "\n"
        "usage: instruction\n"
        "\n"
        " r: Ready to go!\n"
        " s: Stop!\n"
        "\n";*/
/*****************************************
 名称:state_transition
 功能：行为或状态决策
 输入：
 输出：
 *****************************************/
void Decision::state_transition(){
    bool command_ready = 0;
    bool command_run = 0;
    bool command_finish = 0;
    bool command_end = 0;
    for(;;){
        Decision::get_cmd();
        Decision::stateflow();
    }
}

/*****************************************
 名称:get_cmd
 功能：获取当前输入的状态
 输入：
 cmd 键盘输入状态和变量 
 输出：
 
 *****************************************/
void Decision::get_cmd(){
    char cmd[10];
    std::string State_name[5] = {"Driver_mode","Ready_state","Run_state","Finsh_state","Emergency_state"};
    cout << "Current state is: " << State_name[State] << ". ";
    cout << "Please enter your instruction:";
    cin >> cmd;
    cout << "Your instruction is: " << cmd << "\n";

    int i = 0;
    char str_ready[] = "ready",str_run[] = "run",str_finish[] = "fi",str_end[] = "end",str_emergency[] = "e",str_show[]="show";

    if (strcmp(cmd,str_ready)==0)
        i = 1;
    else if (strcmp(cmd,str_run)==0)
        i = 2;
    else if (strcmp(cmd,str_finish)==0)
        i = 3;
    else if (strcmp(cmd,str_end)==0)
        i = 4;
    else if (strcmp(cmd,str_emergency)==0)
        i = 5;
    else if (strcmp(cmd,str_show)==0)
        i = 6;

    switch(i){
        case 1:{
            Command_ready = 1;
            Command_run = 0;
            Command_finish = 0;
            Command_end = 0;
            cout << "Ready Commmand " << endl;
            break;
        }
        case 2:{
            Command_ready = 0;
            Command_run = 1;
            Command_finish = 0;
            Command_end = 0;
            char run_mode[10];
            char str_self[] = "s";
            char str_follow[] = "f";
            cout << "Run Command. Please select mode: self-mode(s) or follow-mode(f):" << endl;
            cin >> run_mode;

            if(strcmp(run_mode,str_self)==0){
                Run_mode_switch = 0;
                string str_speed;
                cout << "Self Mode. Current desired speed is " << Desired_speed << " km/h." << endl;
                cout << "Please enter your desired speed(0~30 km/h):";
                cin >> str_speed;
                int input_speed = stoi(str_speed);
                if(input_speed >= 0 && input_speed <= 30)
                    Desired_speed = input_speed;
                break;
            }//self mode
            else if(strcmp(run_mode,str_follow)==0){
                Run_mode_switch = 1;
                string str_distance;
                cout << "Follow Mode. Current desired distance is " << Desired_distance << "m" << endl;
                cout << "Please enter your desired distance(10~40 m):";
                cin >> str_distance;
                int input_distance = stoi(str_distance);
                if (input_distance >= 10 && input_distance <= 40)
                    Desired_distance = input_distance;
                break;
            }//follow mode
            else {
                cout << "False instruction!" << endl;
                break;
            }
            /*
            try{

            }
             */

            //cout << stoi(str_speed) << endl;//TODO:ADD ERROR HANDLING


        }
        case 3:{
            Command_ready = 0;
            Command_run = 0;
            Command_finish = 1;
            Command_end = 0;
            cout << "Finish Command " << endl;
            break;
        }
        case 4:{
            Command_ready = 0;
            Command_run = 0;
            Command_finish = 0;
            Command_end = 1;
            cout << "End Command " << endl;
            break;
        }
        case 5:{
            Signal_emergency = 1;
            Command_ready = 0;
            Command_run = 0;
            Command_finish = 0;
            Command_end = 0;
            cout << "Emergency signal" << endl;
            break;
        }
        case 6:{
            cout << "Print current state ... " << endl;
            Show_switch = 1;
            //TODO:Add keyboard event to stop print
        }
        default:
            cout << "Please enter instruction.";
    }
}

/*****************************************
 名称:stateflow
 功能：状态机
 输入：
 输出：
 *****************************************/
void Decision::stateflow(){
    switch(State){
        case DRIVER_MODE:{
            if(Command_ready == 1)
                State = READY_STATE;
            else if(Command_end == 1)
                State = DRIVER_MODE;
            break;
        }
        case READY_STATE:{
            if(Signal_emergency == 1)
                State = EMERGENCY_STATE;
            else if(Command_run == 1)
                State = RUN_STATE;
            break;
        }
        case RUN_STATE:{
            if(Signal_emergency == 1)
                State = EMERGENCY_STATE;
            else if(Command_finish == 1)
                State = FINISH_STATE;
            break;
        }
        case FINISH_STATE:{
            if(Command_end== 1)
                State = DRIVER_MODE;
            else if(Command_ready == 1)
                State = READY_STATE;
            break;
        }
        case EMERGENCY_STATE:{
            if (Command_end == 1)
                State = DRIVER_MODE;
            break;
        }
    }
}
