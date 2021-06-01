//
//  feedback.hpp
//  
//
//  Created by ikeda on 2021/05/22.
//

#ifndef feedback_hpp
#define feedback_hpp

#include <stdio.h>
#include <stddef.h> // size_t
#include <deque>
#include <time.h> // clock_t

class PID
{
public:
    double p_gain;
    double i_gain;
    double d_gain;
    
    double pre_err;
    double current_err;
    double integrated_err;
    double differentiated_err;
    
    double ctrl_upper_limit;
    double ctrl_lower_limit;
    double offset;
    clock_t now;
    
    PID();
    PID(double p_
        , double i_
        , double d_
        , double ctrl_upper_limit_
        , double ctrl_lower_limit_
        , double offset_);
    
    void clear_err();
    void push_err(double target, double current, clock_t now_);
    double pop_ctrl();
    void reset_clock(){now = clock();}
};


#endif /* feedback_hpp */
