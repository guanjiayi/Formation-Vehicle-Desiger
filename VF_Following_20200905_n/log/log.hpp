#include<iostream>
#include <mutex>

using namespace std; 

class Log
{
    public :
    Log();
    ~Log();

    void LogUpdate();

    //bool logth_flg;     // 线程执行标志位（logth_flg=fasle,表示现程未执行完,logth_flag=true，表示线程执行完）    

};