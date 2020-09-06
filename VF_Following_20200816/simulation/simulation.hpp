#include <iostream>
using namespace std;

class Simulation
{
    public:
    Simulation();
    ~Simulation();

    static void SimUpdate();

    static bool simth_flag;   //线程执行标志位（simth_flg=fasle,表示现程未执行完,logth_flag=true，表示线程执行完）
};