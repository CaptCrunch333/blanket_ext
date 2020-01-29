#include "looper.hpp"

Looper* Looper::_instance_ptr = NULL;
pthread_mutex_t Looper::lock;

Looper::Looper(){
    _instance_ptr = this;
    if (pthread_mutex_init(&lock, NULL) != 0) { 
        Logger::getAssignedLogger()->log("Mutex Failed to Create ",LoggerLevel::Error);
        //return 1; 
    }
}

void* Looper::Loop1KHz(void *vargp) 
{ 
    Timer* _loop_timer = new Timer();
    
    while(1){
        _loop_timer->tick();
        pthread_mutex_lock(&lock); 
        for (TimedBlock* const& i : _instance_ptr->_timed_blocks){
            if(i->getLoopTime() == block_frequency::hz1000){
                i->loopInternal();
            }
        }
        pthread_mutex_unlock(&lock); 
        int consumed_time =_loop_timer->tockMicroSeconds();
        if (consumed_time>Loop1KHz_dt){
        }
        else{
            usleep(Loop1KHz_dt-consumed_time);
        }
    }
} 

void* Looper::hardwareLoop1KHz(void *vargp) 
{ 
    Timer* _loop_timer = new Timer();

    while(1){
        _loop_timer->tick();
        pthread_mutex_lock(&lock); 
        for (TimedBlock* const& i : _instance_ptr->_timed_blocks){
            if(i->getLoopTime() == block_frequency::hhz1000){
                i->loopInternal();
            }
        }
        pthread_mutex_unlock(&lock); 
        int consumed_time =_loop_timer->tockMicroSeconds();
        if (consumed_time>hardwareLoop1KHz_dt){
        }
        else{
            usleep(hardwareLoop1KHz_dt-consumed_time);
        }
    }
} 

void* Looper::Loop100Hz(void *vargp) 
{ 
    Timer* _loop_timer = new Timer();
    while(1){
        _loop_timer->tick();
        //pthread_mutex_lock(&lock); 
        for (TimedBlock* const& j : _instance_ptr->_timed_blocks){
            if(j->getLoopTime() == block_frequency::hz100){
                j->loopInternal();

            }
        }
        //pthread_mutex_unlock(&lock); 
        int consumed_time =_loop_timer->tockMicroSeconds();
        if (consumed_time>Loop100Hz_dt){
        }
        else{
            usleep(Loop100Hz_dt-consumed_time);
        }
    }
} 

void* Looper::Loop10Hz(void *vargp) 
{
    Timer* _loop_timer = new Timer();
    while(1){
        _loop_timer->tick();
        pthread_mutex_lock(&lock); 
        for (TimedBlock* const& j : _instance_ptr->_timed_blocks){
            if(j->getLoopTime() == block_frequency::hz10){
                j->loopInternal();
            }
        }
        pthread_mutex_unlock(&lock);
        int consumed_time =_loop_timer->tockMicroSeconds();
        if (consumed_time>Loop10Hz_dt){
        }
        else{
            usleep(Loop10Hz_dt-consumed_time);
        }
    }
} 

void Looper::addTimedBlock(TimedBlock* t_cs){
    _timed_blocks.push_back(t_cs);
}
