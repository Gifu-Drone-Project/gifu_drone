//
//  shutdown.cpp
//  
//
//  Created by ikeda on 2021/05/22.
//

#include <stdio.h>
#include "gifu_drone/gifu_drone.hpp"

void GifuDroneMem::shutdown(){
    ROS_INFO("YAMAKAZE> Shutdown Phase.");

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = current_imu_attitude.z;
    ROS_DEBUG("SYSTEM> shutdown: roll:%f pitch:%f yaw:%f\n"
            , roll
            , pitch
            , yaw);
    target_throttle.data = 0.0;
    
    mavros_target.header.stamp = ros::Time::now();
    RollPitchYaw2Quat(mavros_target.orientation, roll, pitch, yaw);
    mavros_target.thrust = target_throttle.data;
    target_pub.publish(mavros_target);

    rate.sleep();

    ROS_INFO("YAMAKAZE> Disarm Vehicle.");
    mavros_msgs::CommandBool disarm_cmd;
    disarm_cmd.request.value = false;
    ros::Time last_request = ros::Time::now();
    while(ros::ok()){
        if(ros::Time::now() - last_request > ros::Duration(5.0)){
            int call_success = arming_client.call(disarm_cmd);
            if(call_success){
                ROS_INFO("SYSTEM> Vehicle disarmed");
                break;
            }
        }
        
        mavros_target.header.stamp = ros::Time::now();
        target_pub.publish(mavros_target);
        
        rate.sleep();
    }
    ROS_INFO("YAMAKAZE> Complete Shutdown.");
    ROS_INFO("YAMAKAZE> Thank you for your hard work.");
}
