//
// Created by Junye Peng on 2026/3/24.
//

#include "StateMachine.hpp"
#include "State.hpp"

#include "State_test.hpp"

StateMachine::StateMachine()
{
    State_test::getInstance();
}

void StateMachine::changeState(State& newState)
{
    if (currentState)
    {
        currentState->exit();
    }

    currentState = &newState;

    if (currentState)
    {
        currentState->init();
    }
}

void StateMachine::stateLoop()
{
    if (currentState)
    {
        currentState->loop();
    }
}
