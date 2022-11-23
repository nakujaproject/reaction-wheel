#ifndef STATE_MACHINE_H
#define STATE_MACHINE_H
#include <rocket_state.h>
class state_machine
{
private:
    RocketState currentState;
public:
    state_machine(void);
    void transition(void);
    RocketState getCurrentState(void);
    //inline RobotState getCurrentState() const { return currentState; }
};

#endif