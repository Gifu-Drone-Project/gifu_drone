//
//  start_and_shutdown.cpp
//  
//
//  Created by ikeda on 2021/05/22.
//

#include <stdio.h>
#include "gifu_drone/gifu_drone.hpp"

void GifuDroneMem::starting(){
    ROS_INFO("YAMAKAZE> Start Initilize Phase.");
    ROS_INFO("YAMAKAZE> Waiting for FCU connection.");
    while(ros::ok() && not fcu_is_connected){
        rate.sleep();
    }

    /*
    tf::StampedTransform initial_transform;
    ROS_INFO("YAMAKAZE> Waiting for Complation tf tree.");
    while(ros::ok()){
        rate.sleep();
        if(updateTransform()) { break; }
    }*/
    
    ROS_INFO("YAMAKAZE> Sending a few setpoints before starting.");
    std::vector<geometry_msgs::Point> initial_position_buff;
    std::vector<std_msgs::Float64> initial_altitude_buff;
    std::vector<geometry_msgs::Vector3> initial_imu_attitude_buff;
    
    bool read_flag = false;
    int break_counter = 0;
    while(!read_flag){
        if(position_initializing_flag == true &&
           altitude_initializing_flag == true &&
           attitude_initializing_flag == true){
            initial_position_buff.push_back(current_position);
            initial_altitude_buff.push_back(current_altitude);
            initial_imu_attitude_buff.push_back(current_imu_attitude);
            position_initializing_flag = false;
            altitude_initializing_flag = false;
            attitude_initializing_flag = false;
            read_flag = true;
        }else{
            rate.sleep();
        }
        if(break_counter > 10000)std::exit(0);
        break_counter ++ ;
    }

    for(int i = 150; ros::ok() && i > 0; --i){
        mavros_target.header.stamp = ros::Time::now();
        RollPitchYaw2Quat(mavros_target.orientation, 0.0, 0.0, current_imu_attitude.z);
        mavros_target.thrust = target_throttle.data;
        target_pub.publish(mavros_target);
        
        rate.sleep();
    }
    
    ROS_INFO("YAMAKAZE> Set Offboard mode.");
    mavros_msgs::SetMode offb_set_mode;
    offb_set_mode.request.custom_mode = "OFFBOARD";
    ros::Time last_request = ros::Time::now();
    
    while(ros::ok() && current_state.mode != "OFFBOARD"){
        
        if(simulation && (ros::Time::now() - last_request > ros::Duration(5.0)) ){
            last_request = ros::Time::now();
            int call_success = set_mode_client.call(offb_set_mode);
            if(call_success && offb_set_mode.response.mode_sent)
            {
                ROS_INFO("SYSTEM> Offboard enabled");
            }
        }

        printf("waiting Offboard mode\n");
        printf("Current is %s\n\n", current_state.mode.c_str());
        /*
        while(ros::ok()){
            mavros_target.header.stamp = ros::Time::now();
            RollPitchYaw2Quat(mavros_target.orientation, 0.0, 0.0, current_imu_attitude.z);
            mavros_target.thrust = target_throttle.data;
            target_pub.publish(mavros_target);

            rate.sleep();
            if(updateTransform()){break;}
        }*/
       
        mavros_target.header.stamp = ros::Time::now();
        RollPitchYaw2Quat(mavros_target.orientation, 0.0, 0.0, current_imu_attitude.z);
        mavros_target.thrust = target_throttle.data;
        target_pub.publish(mavros_target);

        rate.sleep();
    }
    
    while(ros::ok() && rc_arm && not simulation){//ikeda
        ROS_INFO("Reset RC Arm Stick");

        mavros_target.header.stamp = ros::Time::now();
        RollPitchYaw2Quat(mavros_target.orientation, 0.0, 0.0, current_imu_attitude.z);
        mavros_target.thrust = target_throttle.data;
        target_pub.publish(mavros_target);

        rate.sleep();
    }
    
    while(ros::ok() && not rc_arm && not simulation){
        ROS_INFO("Waiting to Arm");
        
        mavros_target.header.stamp = ros::Time::now();
        RollPitchYaw2Quat(mavros_target.orientation, 0.0, 0.0, current_imu_attitude.z);
        mavros_target.thrust = target_throttle.data;
        target_pub.publish(mavros_target);

        rate.sleep();
    }
    
    ROS_INFO("YAMAKAZE> Arm Vehicle.");
    mavros_msgs::CommandBool arm_cmd;
    arm_cmd.request.value = true;
    while(ros::ok() && !current_state.armed){
        
        read_flag = false;
        while(!read_flag){
            if(//position_initializing_flag == true &&
               altitude_initializing_flag == true &&
               attitude_initializing_flag == true){
                initial_position_buff.push_back(current_position);
                initial_altitude_buff.push_back(current_altitude);
                initial_imu_attitude_buff.push_back(current_imu_attitude);
                position_initializing_flag = false;
                altitude_initializing_flag = false;
                attitude_initializing_flag = false;
                read_flag = true;
            }else{
                rate.sleep();
            }
        }

        if(not rc_arm && not simulation){
            ROS_INFO("Waiting to Arm");

            mavros_target.header.stamp = ros::Time::now();
            RollPitchYaw2Quat(mavros_target.orientation, 0.0, 0.0, current_imu_attitude.z);
            mavros_target.thrust = target_throttle.data;
            target_pub.publish(mavros_target);

            rate.sleep();
            continue;
        }
        
        if(ros::Time::now() - last_request > ros::Duration(5.0)){
            last_request = ros::Time::now();
            int call_success = arming_client.call(arm_cmd);
            if(call_success){
                ROS_INFO("SYSTEM> Vehicle armed");
            }
        }
        /*
        while(ros::ok()){
            rate.sleep();
            if(updateTransform()){break;}
        }*/
        mavros_target.header.stamp = ros::Time::now();
        RollPitchYaw2Quat(mavros_target.orientation, 0.0, 0.0, current_imu_attitude.z);
        mavros_target.thrust = target_throttle.data;
        target_pub.publish(mavros_target);

        rate.sleep();
    }
    
    reset_init.data = true;
    reset_init_pub.publish(reset_init);

    initial_position = averagePoint(initial_position_buff);
    initial_altitude = averageFloat64(initial_altitude_buff);
    initial_imu_attitude = averageVector3(initial_imu_attitude_buff);

    target_yaw.data = 0.0;
    target_point.x = 0.0;
    target_point.y = 0.0;
    target_point.z = 0.0;
    
    ROS_DEBUG("SYSTEM> initial_compass_yaw:%f\n", initial_imu_attitude.z);
    ROS_DEBUG("SYSTEM> initial_point: x:%f y:%f z:%f\n"
            , initial_position.x
            , initial_position.y
            , initial_position.z
            );
            

    xp_pid.reset_clock();
    yp_pid.reset_clock();
    zp_pid.reset_clock();
    yaw_pid.reset_clock();
    ROS_INFO("YAMAKAZE> Complete Initilize.");
    ROS_INFO("YAMAKAZE> Good luck.");
}
