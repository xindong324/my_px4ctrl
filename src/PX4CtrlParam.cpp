/**
 * @file PX4CtrlParam.cpp
 * @author your name (you@domain.com)
 * @brief 
 * @version 0.1
 * @date 2022-07-03
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#include "PX4CtrlParam.h"

Parameter_t::Parameter_t()
{

}

/**
 * @brief load ros param from cfg files
 * 
 * @param nh ros nodehandle
 */
void Parameter_t::config_from_ros_handle(const ros::NodeHandle &nh)
{
    read_essential_param(nh, "gain/Kp0", gain.Kp0);
    read_essential_param(nh, "gain/Kp1", gain.Kp1);
    read_essential_param(nh, "gain/Kp2", gain.Kp2);
    read_essential_param(nh, "gain/Kv0", gain.Kv0);
    read_essential_param(nh, "gain/Kv1", gain.Kv1);
    read_essential_param(nh, "gain/Kv2", gain.Kv2);
    read_essential_param(nh, "gain/Kvi0", gain.Kvi0);
    read_essential_param(nh, "gain/Kvi1", gain.Kvi1);
    read_essential_param(nh, "gain/Kvi2", gain.Kvi2);
    read_essential_param(nh, "gain/Kvd0", gain.Kvd0);
    read_essential_param(nh, "gain/Kvd1", gain.Kvd1);
    read_essential_param(nh, "gain/Kvd2", gain.Kvd2);
    read_essential_param(nh, "gain/KAngR", gain.KAngR);
    read_essential_param(nh, "gain/KAngP", gain.KAngP);
    read_essential_param(nh, "gain/KAngY", gain.KAngY);
    read_essential_param(nh, "gain/Ka0", gain.Kv0);
    read_essential_param(nh, "gain/Ka1", gain.Kv1);
    read_essential_param(nh, "gain/Ka2", gain.Kv2);

    read_essential_param(nh, "rotor_drag/x", rt_drag.x);
    read_essential_param(nh, "rotor_drag/y", rt_drag.y);
    read_essential_param(nh, "rotor_drag/z", rt_drag.z);
    read_essential_param(nh, "rotor_drag/k_thrust_horz", rt_drag.k_trust_horz);

    read_essential_param(nh, "msg_timeout/odom", msg_timeout.odom);
    read_essential_param(nh, "msg_timeout/rc", msg_timeout.rc);
    read_essential_param(nh, "msg_timeout/cmd", msg_timeout.cmd);
    read_essential_param(nh, "msg_timeout/imu", msg_timeout.imu);
    read_essential_param(nh, "msg_timeout/bat", msg_timeout.bat);

    read_essential_param(nh, "pose_solver", pose_solver);
    read_essential_param(nh, "mass", mass);
    read_essential_param(nh, "gra", gra);
    read_essential_param(nh, "ctrl_freq_max", ctrl_freq_max);
    read_essential_param(nh, "use_bodyrate_ctrl", use_bodyrate_ctrl);
    read_essential_param(nh, "max_manual_vel", max_manual_vel);
    read_essential_param(nh, "max_angle",max_angle);
    read_essential_param(nh, "low_voltage", low_voltage);

    read_essential_param(nh, "use_yaw_rate_ctrl", use_yawrate_ctrl);
    read_essential_param(nh, "perform_aerodynamics_compensation", perform_aerodynamics_compensation);
    read_essential_param(nh, "attitude_fb", attitude_fb);

    read_essential_param(nh, "hover/use_hov_percent_kf", hover.use_hov_percent_kf);
    read_essential_param(nh, "hover/percent_lower_limit", hover.percent_lower_limit);
    read_essential_param(nh, "hover/percent_higher_limit", hover.percent_higher_limit);


    read_essential_param(nh, "rc_reverse/roll", rc_reverse.roll);
    read_essential_param(nh, "rc_reverse/pitch", rc_reverse.pitch);
    read_essential_param(nh, "rc_reverse/yaw", rc_reverse.yaw);
    read_essential_param(nh, "rc_reverse/throttle", rc_reverse.throttle);

    read_essential_param(nh, "auto_takeoff_land/enable", takeoff_land.enable);
    read_essential_param(nh, "auto_takeoff_land/enable_auto_arm", takeoff_land.enable_auto_arm);
    read_essential_param(nh, "auto_takeoff_land/no_RC", takeoff_land.no_RC);
    read_essential_param(nh, "auto_takeoff_land/height", takeoff_land.height);
    read_essential_param(nh, "auto_takeoff_land/speed", takeoff_land.speed);

    read_essential_param(nh, "thrust_model/print_value", thr_map.print_val);
    read_essential_param(nh, "thrust_model/K1", thr_map.k1);
    read_essential_param(nh, "thrust_model/K2", thr_map.k2);
    read_essential_param(nh, "thrust_model/K3", thr_map.k3);
    read_essential_param(nh, "thrust_model/accurate_thrust_model", thr_map.accurate_thrust_model);
    read_essential_param(nh, "thrust_model/hover_percentage", thr_map.hover_percentage);

    read_essential_param(nh, "full_thrust", full_thrust);
	read_essential_param(nh,"pxy_error_max",pxy_error_max);
	read_essential_param(nh,"vxy_error_max",vxy_error_max);
	read_essential_param(nh,"pz_error_max",pz_error_max);
	read_essential_param(nh,"vz_error_max",vz_error_max);
	read_essential_param(nh,"yaw_error_max",yaw_error_max);

    /* ang in cfg 2 rad in code */
    max_angle /= (180.0/M_PI);

    /* only enable takeoff_land then auto arm can be enabled*/
    if(takeoff_land.enable_auto_arm && !takeoff_land.enable)
    {
        takeoff_land.enable_auto_arm = false;
        ROS_ERROR("\" enabel_auto_arm \" is only allowed with \" auto_takeoff_land \" enabled. ");
    }

    // only enable auto takeoff land and auto arm, then no rc can be enable
    if(takeoff_land.no_RC && (!takeoff_land.enable_auto_arm || !takeoff_land.enable))
    {
        takeoff_land.no_RC = false;
        ROS_ERROR("\"no RC\" is only allowed with both \" auto_takeoff_land\" and \"enable_auto_arm\" enabled.");
    }

    if(thr_map.print_val)
    {
        ROS_WARN("you should disable \"print_value\" if you are in regular usage.");
    }

}

void Parameter_t::init()
{
	full_thrust = mass * gra / thr_map.hover_percentage;
};

void Parameter_t::config_full_thrust(double hov)
{
	full_thrust = hover.use_hov_percent_kf ? (mass * gra / hov) : full_thrust;
};
