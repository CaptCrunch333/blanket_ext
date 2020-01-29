#pragma once
#include <pthread.h>
#include <sched.h>
#include "Timer.hpp"
#include <unistd.h>
const int hardwareLoop1KHz_dt = 5000; //in us
const int Loop1KHz_dt = 5000; //in us
const int Loop100Hz_dt = 10000; //in us
const int Loop10Hz_dt = 100000; //in us
#include "logger.hpp"
#include "TimedBlock.hpp"
#include <list>
#include <iostream>
//TODO: pass _dt to reflect looping rate!!!
class Looper {
    //TODO convert to singleton
    //TODO create Timed Block as parent of ControlSystem

    private:
        std::list<TimedBlock*> _timed_blocks;
        std::list<TimedBlock*>::iterator _it;
        static Looper* _instance_ptr;
        static pthread_mutex_t lock;

    public:
        static void* Loop1KHz(void *vargp);
        static void* Loop100Hz(void *vargp);
        static void* Loop10Hz(void *vargp);
        static void* hardwareLoop1KHz(void *vargp);
        void addTimedBlock(TimedBlock*);

        Looper();
};