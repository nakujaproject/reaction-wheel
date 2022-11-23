#include <state_machine.h>

state_machine::state_machine(void)
{
    currentState = RocketState::idle;
}
RocketState state_machine::getCurrentState(void)
{
    return currentState;
}

void state_machine::transition(void)
{
    currentState = rocketTransitions[currentState];
}

std::map<RocketState, RocketState> rocketTransitions = {
	{RocketState::idle, RocketState::active_flight},
	{RocketState::active_flight, RocketState::post_flight}
};