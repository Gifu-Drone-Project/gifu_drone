/**
 This program was arranged for education of Gifu University reffering https://docs.px4.io/master/en/ros/mavros_offboard.html
 and https://github.com/MeijoDrone
 */
/**
 *  * @file offb_node.cpp
 *   * @brief offboard example node, written with mavros version 0.14.2, px4 flight
 *    * stack and tested in Gazebo SITL
 *     */

#include "gifu_drone/gifu_drone.hpp"

int main(int argc, char **argv){
	std::string target_point_topic("/gdp/target_point");
	std::string target_yaw_topic("/gdp/target_yaw");
	std::string param_xml("./src/gifu_drone/gifu_drone_param.xml");
	cmd_arguments(argc, argv
			, target_point_topic
			, target_yaw_topic
			, param_xml
			);

	ros::init(argc, argv, "gifu_drone_node");
    /*multi thread spinner*/
    ros::AsyncSpinner spinner(12);
    spinner.start();
    
    FILE *fp;
    if((fp=fopen("output.txt","w"))==NULL){
        perror("openfile");
        exit(1);
    }
    
	double control_freq = 200.0;//frequency
    double initial_throttle_average = 0.3;
    
	std::vector<PID> pid_params;
	double throttle_upper_limit, throttle_lower_limit;
    bool simulation;
	bool rc_accept, rc_direct_roll, rc_direct_pitch;
	bool error = loadParamXML(param_xml, pid_params
			, throttle_upper_limit, throttle_lower_limit
            , simulation
			, rc_accept, rc_direct_roll, rc_direct_pitch
			);
	if(error){
		ROS_FATAL("YAMAKAZE> ERROR!! Cannot open Parameter File: %s."
				, param_xml.c_str());
		ROS_FATAL("YAMAKAZE> This system is finished automatically.");
		exit(1);
	}

	GifuDroneMem gdm(
			target_point_topic
			, target_yaw_topic
			, pid_params.at(0) // x
			, pid_params.at(1) // y
			, pid_params.at(2) // alt
			, pid_params.at(3) // yaw
			, throttle_upper_limit
			, throttle_lower_limit
            , simulation
			, rc_accept
			, rc_direct_roll
			, rc_direct_pitch
            , control_freq
            , initial_throttle_average
			);

    ros::Time rostime_getter;
    double tm_init = 0.0;
    double tm_crr = 0.0;
    
	gdm.starting();

	ROS_INFO("YAMAKAZE> Start Main Phase.");
    rostime_getter = ros::Time::now();
    tm_init = (double)(rostime_getter.sec)+(double)(rostime_getter.nsec)/1000000000.0;
    
    fprintf(fp,"xpGp=%lf~xpGi=%lf~xpGd=%lf~",gdm.xp_pid.p_gain,gdm.xp_pid.i_gain,gdm.xp_pid.d_gain);
    fprintf(fp,"\t");
    //
    fprintf(fp,"ypGp=%lf~ypGi=%lf~ypGd=%lf~",gdm.yp_pid.p_gain,gdm.yp_pid.i_gain,gdm.yp_pid.d_gain);
    fprintf(fp,"\t");
    //
    fprintf(fp,"zpGp=%lf~zpGi=%lf~zpGd=%lf~",gdm.zp_pid.p_gain,gdm.zp_pid.i_gain,gdm.zp_pid.d_gain);
    fprintf(fp,"\n");
    
    fprintf(fp,"crrTime\t");
    fprintf(fp,"desX\tdesY\tdesZ\tdesNose\t");
    fprintf(fp,"slamX\tslamY\tlaserZ\tslamNose\t");
    fprintf(fp,"desRoll\tdesPitch\tdesYaw\tdesThrottle\t");
    fprintf(fp,"imuRoll\timuPitch\timuYaw\taveThrottle\t");
    fprintf(fp,"battery\t");
    
    fprintf(fp,"\n");
	while(ros::ok() && gdm.ok() && not gdm.rc_exit){
        rostime_getter = ros::Time::now();
        tm_crr = ((double)(rostime_getter.sec)+(double)(rostime_getter.nsec)/1000000000.0) - tm_init;

//		gdm.updateTransform();
		gdm.updateErr();
		gdm.publishCtrlInput();
        /*time             1*/
        fprintf(fp,"%lf\t",
                tm_crr);
        fprintf(fp,"%lf\t%lf\t%lf\t%lf\t",//desired position and direction
                gdm.target_point.x,//2
                gdm.target_point.y,//3
                gdm.target_point.z,//4
                gdm.target_yaw.data);//5
        fprintf(fp,"%lf\t%lf\t%lf\t%lf\t",//current position and direction
                gdm.current_position.x,//6
                gdm.current_position.y,//7
                gdm.current_altitude.data,//8
                gdm.current_imu_attitude.z);//9
        fprintf(fp,"%lf\t%lf\t%lf\t%lf\t",//desired attitude and throttle
                gdm.roll_ctrl,//10
                gdm.pitch_ctrl,//11
                gdm.yaw_ctrl,//12
                (double)gdm.target_throttle.data);//13
        fprintf(fp,"%lf\t%lf\t%lf\t%lf\t",
                gdm.current_imu_attitude.x,//14
                gdm.current_imu_attitude.y,//15
                gdm.current_imu_attitude.z,//16
                gdm.throttle_ave);//17
        fprintf(fp,"%f\t",
                gdm.battery_percent);//18
        
        fprintf(fp,"\n");
		gdm.rate.sleep();
	}
    fclose(fp);
	gdm.shutdown();

	return 0;
}
