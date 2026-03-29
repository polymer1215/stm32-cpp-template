//
// Created by Junye Peng on 2026/3/25.
//

#ifndef STATE_BALANCE_HPP
#define STATE_BALANCE_HPP

#include "State.hpp"

class State_balance : public State
{
public:
    static State_balance& getInstance()
    {
        static State_balance instance;
        return instance;
    }
    
    void init() override;
    void loop() override;
    void exit() override;

private:
    State_balance() = default;
};

#endif
