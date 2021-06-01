//
//  update_error.cpp
//  
//
//  Created by ikeda on 2021/05/22.
//

#include <stdio.h>
#include "gifu_drone/gifu_drone.hpp"

void GifuDroneMem::updateErr(){
    ros::Time temp = ros::Time::now();
    /*
    tf::Matrix3x3(
            compass_transform.getRotation()).getRPY(
            compass_roll, compass_pitch, compass_yaw);
    tf::Matrix3x3(
            current_transform.getRotation()).getRPY(
            current_roll, current_pitch, current_yaw);
    
    tf::Vector3 t_vec = current_transform.getOrigin();
     */
    // !! Caution !!
    // The coordinate system of PixHawk IMU uses North-East-Down
    // The coordinate system of this controller uses North-West-Up

    clock_t cl = clock();

    fixed_alt = target_point.z;
    if(rc_accept && not simulation){
        if(rc_land){
            landing_alt -= 0.002;
            if(landing_alt>0.0){
                landing_alt = 0.0;
            }else if(landing_alt<-1.0){
                landing_alt = -1.0;
            }
            fixed_alt = landing_alt;
        }else{
            landing_alt=0.0;
            fixed_alt *= rc_throttle_rate;
        }
    }
    
    /* position and velocity directions are inertial coordinate and accele direction is body coordinate */
    xp_pid.push_err(target_point.x, current_position.x, cl);
    yp_pid.push_err(target_point.y, current_position.y, cl);
    zp_pid.push_err(fixed_alt, current_altitude.data,cl);
    printf("target=%lf, current=%lf, p_err=%lf, i_err=%lf, d_err=%lf\n",fixed_alt, current_altitude.data, zp_pid.current_err, zp_pid.integrated_err, zp_pid.differentiated_err);
    yaw_pid.push_err(0, current_imu_attitude.z, cl);
    
    ROS_DEBUG("SYSTEM> initial_point: %f %f %f\n"
            , initial_position.x, initial_position.y, initial_position.z);
    ROS_DEBUG("SYSTEM> target_point: %f %f %f\n"
            , target_point.x, target_point.y, target_point.z);
    ROS_DEBUG("SYSTEM> z position err: P:%f I:%f D:%f\n"
            , zp_pid.current_err, zp_pid.integrated_err
            , zp_pid.differentiated_err);
}
