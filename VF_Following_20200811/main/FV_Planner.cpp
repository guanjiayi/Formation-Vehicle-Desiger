#include "../decision/decision.hpp"
#include "../canmessage/canmessage.hpp"
#include "../control/control.hpp"
#include "../planning/planning.hpp"

int main()
{
    CANMessage ReciveData;
    Decision Decider;
    Planning Planner;
    Control Controler;

    ReciveData.GetCANMessage();
    Decider.CurrentState();
    Planner.PathPlanning();
    Controler.SpeedControl();

    return 0;
}