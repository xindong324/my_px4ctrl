/*
 * @Author: xindong324
 * @Date: 2022-07-02 22:35:00
 * @LastEditors: xindong324
 * @LastEditTime: 2022-09-15 20:05:58
 * @Description: file content
 */
#ifndef __PX4CTRLPARAM_H
#define __PX4CTRLPARAM_H

#include <ros/ros.h>

class Parameter_t
{
private:
    template<typename TName, typename TVal>
    void read_essential_param(const ros::NodeHandle &nh, const TName &name, TVal &val)
    {
        if(nh.getParam(name, val))
        {

        }
        else{
            ROS_ERROR_STREAM("Read param: "<< name<<"failed.");
            // end the program
            ROS_BREAK();
        }
    }
    
public:
    struct Gain
    {
        double Kp0, Kp1, Kp2;
        double Kv0, Kv1, Kv2;
        double Kvi0, Kvi1, Kvi2;
        double Kvd0, Kvd1, Kvd2;
        double Ka0,Ka1,Ka2;
        double KAngR, KAngP, KAngY;
    };

    struct Hover
	{
		bool use_hov_percent_kf;
		
		double percent_lower_limit;
		double percent_higher_limit;
	};

    struct RotorDrag
    {
        double x, y, z;
        double k_trust_horz;
    };

    struct MsgTimeout
    {
        double odom;
        double rc;
        double cmd;
        double imu;
        double bat;
    };

    struct ThrustMapping
    {
        bool print_val;
        double k1;
        double k2;
        double k3;
        bool accurate_thrust_model;
        double hover_percentage;
    };

    struct RCReverse
    {
        bool roll;
        bool pitch;
        bool yaw;
        bool throttle;
    };

    struct AutoTakeoffLand
    {
        bool enable;
        bool enable_auto_arm;
        bool no_RC;
        double height;
        double speed;
    };

    Gain gain;
    //Gain track_gain;
    RotorDrag rt_drag;
    MsgTimeout msg_timeout;
    RCReverse rc_reverse;
    ThrustMapping thr_map;
    AutoTakeoffLand takeoff_land;
    Hover hover;

    int pose_solver;
    int attitude_fb;
    double mass;
    double gra;
    double max_angle;
    double ctrl_freq_max;
    double max_manual_vel;
    double low_voltage;

    double full_thrust;

    bool use_bodyrate_ctrl;
    bool use_yawrate_ctrl;
    bool perform_aerodynamics_compensation;
	double pxy_error_max;
	double vxy_error_max;
	double pz_error_max;
	double vz_error_max;
	double yaw_error_max;

    Parameter_t(/* args */);
    ~Parameter_t(){};
    void init();
    void config_from_ros_handle(const ros::NodeHandle &nh);
    void config_full_thrust(double hov);

};

#endif