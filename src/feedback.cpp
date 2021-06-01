//
//  feedback.cpp
//  
//
//  Created by ikeda on 2021/05/22.
//

#include "gifu_drone/feedback.hpp"
#include <stdio.h>
#include <float.h> // DBL_MIN
#include <math.h>

PID::PID()
: p_gain(1.0)
, i_gain(0.0)
, d_gain(0.0)
, pre_err(0.0)
, current_err(0.0)
, integrated_err(0.0)
, differentiated_err(0.0)
, ctrl_upper_limit(0.0)
, ctrl_lower_limit(0.0)
, offset(0.0)
, now(0)
{
}

PID::PID(double p_, double i_, double d_
         , double ctrl_upper_limit_
         , double ctrl_lower_limit_
         , double offset_
         )
: p_gain(p_)
, i_gain(i_)
, d_gain(d_)
, pre_err(0.0)
, current_err(0.0)
, integrated_err(0.0)
, differentiated_err(0.0)
, ctrl_upper_limit(ctrl_upper_limit_)
, ctrl_lower_limit(ctrl_lower_limit_)
, offset(offset_)
, now(clock())
{
}


void PID::clear_err(){
    integrated_err = 0.0;
}

void PID::push_err(double target, double current, clock_t now_){
    
    double dt_sec = (double)(now_ - now) / (double)(CLOCKS_PER_SEC);
    if(dt_sec == 0.0) { dt_sec = DBL_MIN; }

    double err = target - current;

    // P
    current_err = err;

    // I
    integrated_err += err * dt_sec;
    if(integrated_err < ctrl_lower_limit/i_gain){
        integrated_err = ctrl_lower_limit/i_gain;
    }else if(integrated_err > ctrl_upper_limit/i_gain){
        integrated_err = ctrl_upper_limit/i_gain;
    }

    // D
    differentiated_err = (err - pre_err) / dt_sec;

    now = now_;
    pre_err = err;
}


double PID::pop_ctrl(){
    
    double u = p_gain * current_err
    + i_gain * integrated_err
    + d_gain * differentiated_err
    + offset;
    
    if(u > ctrl_upper_limit){
        u = ctrl_upper_limit;
    }else if(u < ctrl_lower_limit){
        u = ctrl_lower_limit;
    }
    
    return u;
}
