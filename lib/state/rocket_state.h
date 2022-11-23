#ifndef ROCKET_STATES_H
#define ROCKET_STATES_H
#pragma once
#if defined(ESP32) 
	#include <map>
#else
	#include <map>
#endif

enum class RocketState {
	idle,
	active_flight,
    post_flight
};

extern std::map<RocketState, RocketState> rocketTransitions;


#endif 