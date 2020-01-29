#include "TimedBlock.hpp"

TimedBlock::TimedBlock(block_frequency t_bf) {
    _frequency = t_bf;
}

TimedBlock::~TimedBlock() {

}

block_frequency TimedBlock::getLoopTime(){
    return _frequency;
}