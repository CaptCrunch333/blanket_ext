#pragma once
#include "common_types.hpp"

class TimedBlock {

private:
    block_frequency _frequency;

public:

    virtual void loopInternal() = 0;
    block_frequency getLoopTime();

    TimedBlock(block_frequency);
    ~TimedBlock();
};