//
// Created by Junye Peng on 2026/3/24.
//

#ifndef STATE_MACHINE_HPP
#define STATE_MACHINE_HPP

#include "State.hpp"

class StateMachine
{
public:
    static StateMachine& getInstance()
    {
        static StateMachine instance;
        return instance;
    }

    StateMachine();
    void changeState(State& newState);
    void stateLoop();
private:
    State* currentState = nullptr;
};

#endif
