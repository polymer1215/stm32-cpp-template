#ifndef STATE_HPP
#define STATE_HPP

class State {
public:
    State() {};
    virtual ~State() = default;
    virtual void init() = 0;
    virtual void loop() = 0;
    virtual void exit() = 0;
};

#endif