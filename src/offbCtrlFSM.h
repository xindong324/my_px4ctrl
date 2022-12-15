#ifndef __PX4CTRLFSM_H
#define __PX4CTRLFSM_H

#include <ros/ros.h>
#include <ros/assert.h>

#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>

#include "input.h"
#include "nlcontroller.h"

struct AutoTakeoffLand_t
{
    bool landed{true};
    ros::Time toggle_takeoff_land_time;
    std::pair<bool, ros::Time> delay_trigger{std::pair<bool, ros::Time>(false, ros::Time(0))};
    Eigen::Vector4d start_pose;

    static constexpr double MOTORS_SPEEDUP_TIME = 3.0; // motor spin running for 3 seconds before takeoff
    static constexpr double DELAY_TRIGGER_TIME = 2.0; // time to be delayed when reach at target height
};

class OffbCtrlFSM
{
private:
    /* data */
public:
    Parameter_t &param_;
    RC_Data_t rc_data_;
    State_Data_t state_data_;
    ExtendedState_Data_t extended_state_data_;
    Emergency_Landing_t emergency_landing_;

    Odom_Data_t odom_data_;
    Imu_Data_t imu_data_;
    Command_Data_t command_data_;
    Battery_Data_t battery_data_;
    Takeoff_Land_Data_t takeoff_land_data_;
    Keyboard_t keyboard_data_;

    Controller &controller_;

    ros::Publisher traj_start_trigger_pub_;
    ros::Publisher ctrl_FCU_pub_;
    ros::Publisher debug_pub_;
    ros::ServiceClient set_FCU_mode_srv_;
    ros::ServiceClient arming_client_srv_;
    ros::ServiceClient reboot_FCU_srv_;

    quadrotor_msgs::Px4ctrlDebug debug_msg_;
    Eigen::Vector4d hover_pose_;
    ros::Time last_set_hover_pose_time_;

    enum State_t{
        INIT = 1,
        AUTO_HOVER,
        CMD_CTRL,
        AUTO_TAKEOFF,
        AUTO_LAND,
        EMERGENCY_STOP
    };

    OffbCtrlFSM(Parameter_t &, Controller &);
    // ~PX4CtrlFSM();

    void process();
    bool rc_is_received(const ros::Time &now_time);
    bool cmd_is_received(const ros::Time &now_time);
    bool odom_is_received(const ros::Time &now_time);
    bool imu_is_received(const ros::Time &now_time);
    bool bat_is_received(const ros::Time &now_time);
    bool recv_new_odom();
    State_t get_state(){return state_;}
    bool get_landed(){return takeoff_land_.landed;}

private:
    State_t state_; // only be changed in process funciton
    AutoTakeoffLand_t takeoff_land_;

    // -- control related
    Desired_State_t get_hover_des();
    Desired_State_t get_cmd_des();

    // auto takeoff land
    void motors_idling(const Imu_Data_t &imu, Controller_Output_t &u);
    void land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom); // detect landing
    void set_start_pose_for_takeoff_land(const Odom_Data_t &odom);
    Desired_State_t get_rotor_speed_up_des(const ros::Time &now);
    Desired_State_t get_takeoff_land_des(const double speed);

    // -- tools 
    void set_hov_with_odom();
    void set_hov_with_rc();

    bool toggle_offboard_mode(bool on_off); // it will only try to toggle onece so not blocked
    bool toggle_arm_disarm(bool arm);
    void reboot_FCU();

    void publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
    void publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp);
    void publish_trigger(const nav_msgs::Odometry &odom_msg);

};


#endif