//
//  gifu_drone.hpp
//  
//
//  Created by ikeda on 2021/05/22.
//

#ifndef gifu_drone_h
#define gifu_drone_h

#include <math.h> // atan2()
#include <limits.h> // isnan()
#include <stdlib.h>
#include <string>
#include <vector>
#include <time.h> // clock_t
#include <tinyxml.h>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Float64.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Int16.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <sensor_msgs/Imu.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/AttitudeTarget.h>
#include <tf2/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/WrenchStamped.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/BatteryState.h>

#include "gifu_drone/feedback.hpp"
#include "gifu_drone/functions.hpp"

class GifuDroneMem{
public:
    //system
    ros::NodeHandle nh;
    ros::Rate rate;
    mavros_msgs::State current_state;

    //subscribers
    ros::Subscriber state_sub;
    ros::Subscriber target_point_sub;
    ros::Subscriber target_yaw_sub;
    ros::Subscriber quit_sub;
    ros::Subscriber rc_sub;
    ros::Subscriber battery_sub;
    ros::Subscriber position_sub;
    ros::Subscriber altitude_sub;
    ros::Subscriber attitude_sub;
    ros::Subscriber pose_sub;//for simulation
    
    //for tf
    ros::Publisher reset_init_pub;
    std_msgs::Bool reset_init;
    
    //order to Pixhawk
    ros::Publisher target_pub;
    ros::ServiceClient arming_client;
    ros::ServiceClient set_mode_client;
    
    //radio controller
    mavros_msgs::RCIn rc_raw;
    bool rc_accept;
    bool rc_direct_roll;
    bool rc_direct_pitch;
    double rc_roll;
    double rc_pitch;
    double rc_yaw;
    double rc_throttle_rate;
    bool rc_land;
    bool rc_exit;
    bool rc_interrupt;
    bool rc_override;
    bool rc_arm;
    
    //controll variables
    double roll_ctrl;
    double pitch_ctrl;
    double yaw_ctrl;
    double fixed_alt;
    double throttle_ctrl;
    double throttle_ave;
    double landing_alt;

    //position and attitude
    geometry_msgs::Point current_position;
    geometry_msgs::Point initial_position;
    std_msgs::Float64 current_altitude;
    std_msgs::Float64 initial_altitude;
    geometry_msgs::Vector3 current_imu_attitude;
    geometry_msgs::Vector3 initial_imu_attitude;
    bool position_initializing_flag;
    bool altitude_initializing_flag;
    bool attitude_initializing_flag;
    
    geometry_msgs::Point target_point;
    std_msgs::Float64 target_yaw;
    std_msgs::Float64 target_throttle;
    mavros_msgs::AttitudeTarget mavros_target;
       
    float battery_percent;
    
    bool simulation;
    PID xp_pid;
    PID yp_pid;
    PID zp_pid;
    PID yaw_pid;
    double throttle_upper_limit;
    double throttle_lower_limit;

    bool quit_flg;
    bool fcu_is_connected;

    tf::TransformListener listener;
    tf::StampedTransform current_transform;
    tf::StampedTransform compass_transform;
    
    GifuDroneMem(const std::string &target_point_topic
                 , const std::string &target_yaw_topic
                 , const PID &xp_pid_
                 , const PID &yp_pid_
                 , const PID &zp_pid_
                 , const PID &yaw_pid_
                 , double throttle_upper_limit_
                 , double throttle_lower_limit_
                 , bool simulation_
                 , bool rc_accept_
                 , bool rc_direct_roll_
                 , bool rc_direct_pitch_
                 , double control_freq_
                 , double initial_throttle_average_
                 );
    
    void starting();
    void shutdown();
    
    void updateErr();
    void publishCtrlInput();

    bool ok(){
        return (! quit_flg) && current_state.armed;
    }

    void state_cb(const mavros_msgs::StateConstPtr &msg){
        current_state = *msg;
        fcu_is_connected = current_state.connected;
    }
    
    bool updateTransform(){
        try{
            listener.lookupTransform("map", "base_frame", ros::Time(0), current_transform);
                listener.lookupTransform("map", "fc_imu", ros::Time(0), compass_transform);
            }
            catch(tf::TransformException ex){
                ROS_ERROR("YAMAKAZE> %s.", ex.what());
                return false;
            }
        return true;
    }
    
    void target_point_cb(const geometry_msgs::PointConstPtr &msg){
        target_point.x = msg->x;
        target_point.y = msg->y;
        target_point.z = msg->z;
    }

    void target_yaw_cb(const std_msgs::Float64ConstPtr &msg){
        target_yaw.data = msg->data;
        
    }

    void quit_cb(const std_msgs::EmptyConstPtr &msg){
        quit_flg = true;
    }

    void rc_cb(const mavros_msgs::RCInConstPtr &msg){
        /*     pitch            throttle        */
        /*          1094              1934      */
        /*          ^                 ^         */
        /*          | yaw             | roll     */
        /*   1094 <-o-> 1924   1094 <-o-> 1934  */
        /*          |                 |         */
        /*          v                 v         */
        /*          1934              1094      */
        /*                                      */
        /* RC Joystic Layout and boundary value */

        if(rc_accept){
            rc_raw = *msg;
            
            if(rc_raw.channels[13] > 1450){
                rc_land = true;
            }else{
                rc_land = false;
            }
                         
            if((not rc_override) and rc_raw.channels[8] > 1450){
                rc_override = true;
            }
            if(rc_override and rc_raw.channels[8] < 1450){
                rc_override = false;
            }

            if((not rc_override)
               and (
                    ((not rc_direct_roll)
                     and fabs(rc_raw.channels[0] - 1514) > 50)
                    or
                    ((not rc_direct_pitch)
                     and fabs(rc_raw.channels[1] - 1514) > 50))){
                        rc_interrupt = true;
                    }
            rc_roll = remap((double)(rc_raw.channels[0])
                            , 1934, 1094
                            , 7.0 * M_PI / 180.0
                            , -7.0 * M_PI / 180.0);
            
            rc_pitch = -1.0 * remap((double)(rc_raw.channels[1])
                                    , 1934, 1094
                                    , 7.0 * M_PI / 180.0
                                    , -7.0 * M_PI / 180.0);
            
            rc_yaw = -1.0 * remap((double)(rc_raw.channels[3])
                                  , 1934, 1094
                                  , 7.0 * M_PI / 180.0
                                  , -7.0 * M_PI / 180.0);
            
            if(rc_yaw >= M_PI / 2){
                rc_yaw = M_PI / 2;
            }
            if(rc_yaw <= -M_PI / 2){
                rc_yaw = -M_PI / 2;
            }
                
            rc_throttle_rate = remap((double)(rc_raw.channels[5])
                                     , 1745, 1283
                                     , 1.0 , 0.0);
            
            if(rc_raw.channels[6] > 1450){
                rc_arm = true;
                rc_exit = false;
            }else{
                rc_arm = false;
                rc_exit = true;
            }
        }
    }

    void battery_cb(const sensor_msgs::BatteryStateConstPtr &msg){
        battery_percent = msg->percentage;
    }
    
    void position_cb(const geometry_msgs::PoseStamped &msg){
        position_initializing_flag = true;
        current_position = msg.pose.position;
    }

    void altitude_cb(const std_msgs::Float64 &msg){
        altitude_initializing_flag = true;
        current_altitude.data = msg.data;
    }
    
    void attitude_cb(const sensor_msgs::Imu& msg){
        attitude_initializing_flag = true;
        Quat2RollPitchYaw(msg.orientation, current_imu_attitude.x, current_imu_attitude.y, current_imu_attitude.z);
    }
    
    void pose_cb(const geometry_msgs::PoseStamped &msg){
        if(simulation){
            position_initializing_flag = true;
            altitude_initializing_flag = true;
            attitude_initializing_flag = true;

            current_position = msg.pose.position;
            current_altitude.data = msg.pose.position.z;
            Quat2RollPitchYaw(msg.pose.orientation, current_imu_attitude.x, current_imu_attitude.y, current_imu_attitude.z);
        }
    }
};

void readme(char *name);
void cmd_arguments(
        int argc
        , char **argv
        , std::string &target_point_topic
        , std::string &target_yaw_topic
        , std::string &param_xml
                   );
bool loadParamXML(const std::string &xml_path
        , std::vector<PID> &params
        , double &throttle_upper_limit
        , double &throttle_lower_limit
        , bool &simulation
        , bool &rc_accept
        , bool &rc_direct_roll
        , bool &rc_direct_pitch
                  );

#endif /* gifu_drone_h */
