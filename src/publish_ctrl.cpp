//
//  publish_ctrl.cpp
//  
//
//  Created by ikeda on 2021/05/22.
//

#include <stdio.h>
#include "gifu_drone/gifu_drone.hpp"

void GifuDroneMem::publishCtrlInput(){
    // !! Caution !!
    // The coordinate system of PixHawk IMU uses North-East-Down
    // The coordinate system of the controller uses North-West-Up
    
    /* position and velocity directions are inertial coordinate and accele direction is body coordinate */
    /* considering yaw rotation */
    double x_fb_tmp = xp_pid.pop_ctrl();
    double y_fb_tmp = yp_pid.pop_ctrl();
    double yaw_tmp = current_imu_attitude.z - initial_imu_attitude.z/*-current_yaw*//*compass_yaw -initial_compass_yaw*/;
    
    //    = (rc_direct_roll ? rc_roll : -1.0 * yp_pid.pop_ctrl() - yv_pid.pop_ctrl() - ya_pid.pop_ctrl()  );
    /*  considering yaw rotation */
    //    = (rc_direct_pitch ? rc_pitch :  xp_pid.pop_ctrl() + xv_pid.pop_ctrl() + xa_pid.pop_ctrl() );
    /*  considering yaw rotation */

    pitch_ctrl = (rc_direct_pitch ? rc_pitch :        ( x_fb_tmp * cos(yaw_tmp) - y_fb_tmp * sin(yaw_tmp) ) );
    roll_ctrl  = (rc_direct_roll  ? rc_roll  : -1.0 * ( y_fb_tmp * cos(yaw_tmp) + x_fb_tmp * sin(yaw_tmp) ) );

    yaw_ctrl = yaw_pid.pop_ctrl();

    throttle_ctrl         = zp_pid.pop_ctrl();
    target_throttle.data = throttle_ctrl + throttle_ave;

    
    //throttle_ave is average throttle during hover in stable altitude
    if(current_state.mode == "OFFBOARD"){
        throttle_ave = 0.999 * throttle_ave + 0.001 * ( target_throttle.data - zp_pid.offset );
        if(throttle_ave > throttle_upper_limit){
            throttle_ave = throttle_upper_limit;
        }else if(throttle_ave < throttle_lower_limit){
            throttle_ave = throttle_lower_limit;
        }
    }
        
    if(rc_accept and (rc_override or rc_interrupt)){
        if(rc_override){ROS_WARN("YAMAKAZE> You have control.");}
        if(rc_interrupt){ROS_FATAL("YAMAKAZE> YOU HAVE CONTROL!!");}
        roll_ctrl = rc_roll;
        pitch_ctrl = rc_pitch;
        yaw_ctrl = rc_yaw;
    }

    if(target_throttle.data > 1.0){target_throttle.data = 1.0;}
    if(target_throttle.data < 0.0){target_throttle.data = 0.0;}
    
    ROS_DEBUG("SYSTEM> target throttle:%f\n"
            , target_throttle.data);
    ROS_DEBUG("SYSTEM> target att roll:%f, pitch:%f, yaw:%f\n"
            , roll_ctrl
            , pitch_ctrl
            , yaw_ctrl);
    
    mavros_target.header.stamp = ros::Time::now();
    RollPitchYaw2Quat(mavros_target.orientation
            , roll_ctrl
            , pitch_ctrl
            , /*compass_yaw*/ current_imu_attitude.z + yaw_ctrl);
    mavros_target.thrust = target_throttle.data;
    target_pub.publish(mavros_target);

}
