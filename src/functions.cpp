//
//  functions.cpp
//  
//
//  Created by IkedaGifu on 2021/05/24.
//

#include "gifu_drone/functions.hpp"

void Quat2RollPitchYaw(const geometry_msgs::Quaternion &q, double &roll, double &pitch, double &yaw){
    tf2::Quaternion q_tf2(q.x, q.y, q.z, q.w);
    tf2::Matrix3x3(q_tf2).getRPY(roll, pitch, yaw);
}

void RollPitchYaw2Quat(geometry_msgs::Quaternion &q, double roll, double pitch, double yaw){
    tf2::Quaternion q_tf2(q.x, q.y, q.z, q.w);
    q_tf2.setRPY(roll, pitch, yaw);
    q.x = q_tf2.x();
    q.y = q_tf2.y();
    q.z = q_tf2.z();
    q.w = q_tf2.w();
}

//value of radio control -> desired values, angle and so on
double remap(double in
             , double in_upper_lim, double in_lower_lim
             , double out_upper_lim, double out_lower_lim)
{
    double ret =  (in - in_lower_lim)
    * (out_upper_lim - out_lower_lim)
    / (in_upper_lim - in_lower_lim)
    + out_lower_lim;
    if(ret < out_lower_lim){ret = out_lower_lim;}
    if(ret > out_upper_lim){ret = out_upper_lim;}
    return ret;
}

void TimeSet::update(){
    raw = ros::Time::now();
    if(initial == 0.0){
        initial = (double)(raw.sec)+(double)(raw.nsec)/1000000000.0;
        current = 0.0;
        past    = 0.0;
    }else{
        past = current;
        current = (double)(raw.sec)+(double)(raw.nsec)/1000000000.0 - initial;
        if((current-past) < limit){
            diff = 0.9*diff + 0.1*(current - past);
        }
    }
}

void my_sum(geometry_msgs::Point &out, geometry_msgs::Point &in){
    out.x += in.x;
    out.y += in.y;
    out.z += in.z;
}
void my_sum(geometry_msgs::Vector3 &out, geometry_msgs::Vector3 &in){
    out.x += in.x;
    out.y += in.y;
    out.z += in.z;
}
void my_sum(std_msgs::Float64 &out, std_msgs::Float64 &in){
    out.data += in.data;
}

double averageValue(const std::vector<double> &values){
    double ave = 0.0;
    size_t n = 0;
    for(auto value= values.begin(); value != values.end(); ++value){
        if(std::isnan(*value)){continue;}
        ave += *value;
        ++n;
    }
    ave /= (double)(n);

    return ave;
}

std_msgs::Float64 averageFloat64(const std::vector<std_msgs::Float64> &values){
    double data = 0.0;
    size_t n_data = 0;
    for(auto v = values.begin(); v != values.end(); ++v){
        if(std::isnan(v->data)){continue;}
        data += v->data;
        ++n_data;
    }
    data /= (double)(n_data);
    std_msgs::Float64 ave;
    ave.data = data;
    return ave;
}

geometry_msgs::Point averagePoint(const std::vector<geometry_msgs::Point> &points){
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    size_t n_data = 0;
    for(auto p = points.begin(); p != points.end(); ++p){
        if(std::isnan(p->x) or std::isnan(p->y) or std::isnan(p->z)){continue;}
        x += p->x;
        y += p->y;
        z += p->z;
        ++n_data;
    }
    x /= (double)(n_data);
    y /= (double)(n_data);
    z /= (double)(n_data);
    geometry_msgs::Point ave;
    ave.x = x;
    ave.y = y;
    ave.z = z;
    return ave;
}

geometry_msgs::Vector3 averageVector3(const std::vector<geometry_msgs::Vector3> &points){
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    size_t n_data = 0;
    for(auto p = points.begin(); p != points.end(); ++p){
        if(std::isnan(p->x) or std::isnan(p->y) or std::isnan(p->z)){continue;}
        x += p->x;
        y += p->y;
        z += p->z;
        ++n_data;
    }
    x /= (double)(n_data);
    y /= (double)(n_data);
    z /= (double)(n_data);
    geometry_msgs::Vector3 ave;
    ave.x = x;
    ave.y = y;
    ave.z = z;
    return ave;
}
