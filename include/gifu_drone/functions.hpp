//
//  functions.hpp
//  
//
//  Created by IkedaGifu on 2021/05/24.
//

#ifndef functions_hpp
#define functions_hpp

#include <stdio.h>
#include <math.h>
#include <stdlib.h>
#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Point.h>
#include <tf2/LinearMath/Transform.h>
#include <tf/transform_listener.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/RCIn.h>
#include <std_msgs/Float64.h>

void Quat2RollPitchYaw(const geometry_msgs::Quaternion &q, double &roll, double &pitch, double &yaw);

void RollPitchYaw2Quat(geometry_msgs::Quaternion &q, double roll, double pitch, double yaw);

double remap(double in, double in_upper_lim, double in_lower_lim, double out_upper_lim, double out_lower_lim);


class TimeSet{
public:
    ros::Time raw;
    double initial;
    double current;
    double past;
    double diff;
    double limit;
    TimeSet(double diff_){
        initial = 0.0;
        current = 0.0;
        past    = 0.0;
        diff    = diff_;
        limit   = 1.0;
    }
    TimeSet(double diff_, double limit_){
        initial = 0.0;
        current = 0.0;
        past    = 0.0;
        diff    = diff_;
        limit   = limit_;
    }
    void update();
};

void sum_vector(geometry_msgs::Point &out, geometry_msgs::Point &in);
void sum_vector(geometry_msgs::Vector3 &out, geometry_msgs::Vector3 &in);
void my_sum(std_msgs::Float64 &out, std_msgs::Float64 &in);

double averageValue(const std::vector<double> &values);
std_msgs::Float64 averageFloat64(const std::vector<std_msgs::Float64> &values);
geometry_msgs::Point averagePoint(const std::vector<geometry_msgs::Point> &points);
geometry_msgs::Vector3 averageVector3(const std::vector<geometry_msgs::Vector3> &points);

#endif /* functions_hpp */
