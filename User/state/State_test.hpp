//
// Created by Junye Peng on 2026/3/24.
//
#ifndef STATE_TEST_HPP
#define STATE_TEST_HPP

#include "State.hpp"

class State_test : public State
{
public:
    static State_test& getInstance()
    {
        static State_test instance;
        return instance;
    }

    State_test();
    void init() override;
    void loop() override;
    void exit() override;

private:
    static void timerTaskTest1();
    static void timerTaskTest2();
    static void timerTaskTest3();

};

#endif
