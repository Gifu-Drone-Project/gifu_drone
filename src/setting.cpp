//
//  setting.cpp
//  
//
//  Created by IkedaGifu on 2021/06/01.
//

#include "gifu_drone/gifu_drone.hpp"

GifuDroneMem::GifuDroneMem(
    const std::string &target_point_topic
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
  )
: recognition_marker_id(0)
, nh()
, rate(control_freq_) // must faster than 2 Hz
, current_state()

, state_sub()
, target_point_sub()
, target_yaw_sub()
, quit_sub()
, rc_sub()
, battery_sub()
, position_sub()
, altitude_sub()
, attitude_sub()
, pose_sub()

, reset_init_pub()
, reset_init()
, target_pub()
, arming_client()
, set_mode_client()

, simulation(simulation_)
, rc_raw()
, rc_accept(rc_accept_)
, rc_direct_roll(rc_direct_roll_)
, rc_direct_pitch(rc_direct_pitch_)
, rc_throttle_rate(1.0)
, rc_roll(0.0)
, rc_pitch(0.0)
, rc_yaw(0.0)
, rc_land(false)
, rc_exit(false)
, rc_interrupt(false)
, rc_override(false)
, rc_arm(false)

, roll_ctrl(0.0)
, pitch_ctrl(0.0)
, yaw_ctrl(0.0)
, fixed_alt(0.0)
, throttle_ctrl(0.0)
, throttle_ave(initial_throttle_average_)
, landing_alt(0.0)

, current_position()
, initial_position()
, current_altitude()
, initial_altitude()
, current_imu_attitude()
, initial_imu_attitude()
, position_initializing_flag(false)
, altitude_initializing_flag(false)
, attitude_initializing_flag(false)

, target_point()
, target_yaw()
, mavros_target()

, battery_percent(0.0)

, xp_pid(xp_pid_)
, yp_pid(yp_pid_)
, zp_pid(zp_pid_)
, yaw_pid(yaw_pid_)
, throttle_upper_limit(throttle_upper_limit_)
, throttle_lower_limit(throttle_lower_limit_)

, quit_flg(false)
, fcu_is_connected(false)
, listener()
, current_transform()
, compass_transform()
{
    state_sub
        = nh.subscribe<mavros_msgs::State>(
                "mavros/state", 10, &GifuDroneMem::state_cb, this);
    target_point_sub
        = nh.subscribe<geometry_msgs::Point>(
                target_point_topic.c_str(), 10
                , &GifuDroneMem::target_point_cb, this);
    target_yaw_sub
        = nh.subscribe<std_msgs::Float64>(
                target_yaw_topic.c_str(), 10
                , &GifuDroneMem::target_yaw_cb, this);
    quit_sub
        = nh.subscribe<std_msgs::Empty>(
                "/gdp/quit", 10
                , &GifuDroneMem::quit_cb, this);
    rc_sub
        = nh.subscribe<mavros_msgs::RCIn>(
                "/mavros/rc/in", 10
                , &GifuDroneMem::rc_cb, this);

    battery_sub
        = nh.subscribe<sensor_msgs::BatteryState>(
                "/mavros/battery", 10
                , &GifuDroneMem::battery_cb, this);
    
    position_sub = nh.subscribe(
                /*"/gdp/position"*/"/xyz_difference", 10
                , &GifuDroneMem::position_cb, this);

    altitude_sub
        = nh.subscribe(
                /*"/gdp/altitude"*/"/teraranger_evo_mini/range", 10
                , &GifuDroneMem::altitude_cb, this);
    
    attitude_sub
        = nh.subscribe(
                "/mavros/imu/data", 10
                , &GifuDroneMem::attitude_cb, this);
    
    pose_sub
        = nh.subscribe(
                "/mavros/local_position/pose", 10
                , &GifuDroneMem::pose_cb, this);
    
    reset_init_pub = nh.advertise<std_msgs::Bool>("/gdp/reset_initial", 1);
        
    target_pub = nh.advertise<mavros_msgs::AttitudeTarget>("mavros/setpoint_raw/attitude",100);
    
    arming_client
        = nh.serviceClient<mavros_msgs::CommandBool>(
                "mavros/cmd/arming");
    set_mode_client
        = nh.serviceClient<mavros_msgs::SetMode>(
                "mavros/set_mode");
    
    ROS_INFO("YAMAKAZE> Set PID Parameters.");
    ROS_DEBUG("SYSTEM> position Pgain Igain Dgain PreDgain HighLimit LowLimit\n");
    ROS_DEBUG("X %f %f %f %f %f\n", xp_pid.p_gain, xp_pid.i_gain, xp_pid.d_gain, xp_pid.ctrl_upper_limit, xp_pid.ctrl_lower_limit);
    ROS_DEBUG("Y %f %f %f %f %f\n", yp_pid.p_gain, yp_pid.i_gain, yp_pid.d_gain, yp_pid.ctrl_upper_limit, yp_pid.ctrl_lower_limit);
    ROS_DEBUG("Z %f %f %f %f %f\n", zp_pid.p_gain, zp_pid.i_gain, zp_pid.d_gain, zp_pid.ctrl_upper_limit, zp_pid.ctrl_lower_limit);
    ROS_DEBUG("YAW %f %f %f %f %f\n", yaw_pid.p_gain, yaw_pid.i_gain, yaw_pid.d_gain, yaw_pid.ctrl_upper_limit, yaw_pid.ctrl_lower_limit);

    ROS_INFO("YAMAKAZE> Set Machine Parameter.");

}



void readme(char *name){
    ROS_INFO("SYSTEM>\n\
            Usage: %s \n\
            -t <target_point_topic [geometry_msgs::Point in]>\n\
            -y <target_yaw_topic [std_msgs::Float64 in]> \n\
            -f <parameter xml file> \n\
            \n\
            Ex: %s \n\
            -t /gdp/target_point\n\
            -y /gdp/target_yaw\n\
            -f param_RW_quad.xml\n\
            \n", name, name);
}

void cmd_arguments(
        int argc
        , char **argv
        , std::string &target_point_topic
        , std::string &target_yaw_topic
        , std::string &param_xml
        ){
    int result;
    while((result = getopt(argc, argv, "ht:y:f:")) != -1){
        switch(result){
            case 'h':
                readme(argv[0]);
                exit(0);
                break;
            case 't':
                target_point_topic = std::string(optarg);
                break;
            case 'y':
                target_yaw_topic = std::string(optarg);
                break;
            case 'f':
                param_xml = std::string(optarg);
                break;
        }
    }
}

bool loadParamXML(const std::string &xml_path
        , std::vector<PID> &params
        , double &throttle_upper_limit
        , double &throttle_lower_limit
        , bool &simulation
        , bool &rc_accept
        , bool &rc_direct_roll
        , bool &rc_direct_pitch
        ){
    params.clear();
    params.resize(10);
    TiXmlDocument xml(xml_path.c_str());
    bool success = xml.LoadFile();
    if(not success){return 1;}
    TiXmlElement *data = xml.FirstChildElement("param");

    TiXmlElement *machine_param = data->FirstChildElement("machine");
    simulation
        = (strncmp(machine_param->FirstChildElement(
                    "simulation")->GetText(), "true", 4) == 0);
    throttle_upper_limit
        = atof(machine_param->FirstChildElement(
                    "throttle_upper_limit")->GetText());
    throttle_lower_limit
        = atof(machine_param->FirstChildElement(
                    "throttle_lower_limit")->GetText());
    rc_accept
        = (strncmp(machine_param->FirstChildElement(
                    "rc_accept")->GetText(), "true", 4) == 0);
    rc_direct_roll
        = (strncmp(machine_param->FirstChildElement(
                    "rc_direct_roll")->GetText(), "true", 4) == 0);
    rc_direct_pitch
        = (strncmp(machine_param->FirstChildElement(
                    "rc_direct_pitch")->GetText(), "true", 4) == 0);

    TiXmlElement *pid_param = data->FirstChildElement("pid");
    TiXmlElement *elm[3], *yaw_elm;
    elm[0] = pid_param->FirstChildElement("x");
    elm[1] = pid_param->FirstChildElement("y");
    elm[2] = pid_param->FirstChildElement("z");
    yaw_elm = pid_param->FirstChildElement("yaw");

    for(size_t i = 0; i < 3; ++i){
        TiXmlElement *position, *velocity, *accel;
        position = elm[i]->FirstChildElement("position");
        params.at(i)
            = PID(
                    atof(position->FirstChildElement("p_gain")->GetText())
                    , atof(position->FirstChildElement("i_gain")->GetText())
                    , atof(position->FirstChildElement("d_gain")->GetText())
                    , atof(position->FirstChildElement("upper_limit")->GetText())
                    , atof(position->FirstChildElement("lower_limit")->GetText())
                    , atof(position->FirstChildElement("offset")->GetText())
                 );
    }
    params.at(3)
        = PID(
                atof(yaw_elm->FirstChildElement("p_gain")->GetText())
                , atof(yaw_elm->FirstChildElement("i_gain")->GetText())
                , atof(yaw_elm->FirstChildElement("d_gain")->GetText())
                , atof(yaw_elm->FirstChildElement("upper_limit")->GetText())
                , atof(yaw_elm->FirstChildElement("lower_limit")->GetText())
                , atof(yaw_elm->FirstChildElement("offset")->GetText())
             );
    return 0;
}
