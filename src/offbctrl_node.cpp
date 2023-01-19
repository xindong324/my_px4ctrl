#include <ros/ros.h>
#include "offbCtrlFSM.h"
#include <signal.h>

void mySigintHandler(int sig)
{
    ROS_INFO("[PX4Ctrl] exit...");
    ros::shutdown();
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "px4ctrl");
    ros::NodeHandle nh("~");

    signal(SIGINT, mySigintHandler);
    ros::Duration(1.0).sleep();

    Parameter_t param;
    param.config_from_ros_handle(nh);

    // Controller controller(param);
    Controller controller(param);
    HovThrKF hov_thr_kf(param);
    OffbCtrlFSM fsm(param, controller,hov_thr_kf);

    fsm.hov_thr_kf_.init();
    fsm.hov_thr_kf_.set_hov_thr(param.thr_map.hover_percentage);//x(0) = hov_percent
    
    ROS_INFO("Initial value for hov_thr set to %.2f/%.2f",
             fsm.hov_thr_kf_.get_hov_thr(),
             param.mass * param.gra / param.full_thrust);
    ROS_INFO("Hovering thrust kalman filter is %s.",
             param.hover.use_hov_percent_kf ? "used" : "NOT used");

    ros::Subscriber state_sub =
        nh.subscribe<mavros_msgs::State>("/mavros/state",
                                         10,
                                         boost::bind(&State_Data_t::feed, &fsm.state_data_, _1));

    ros::Subscriber extended_state_sub =
        nh.subscribe<mavros_msgs::ExtendedState>("/mavros/extended_state",
                                                 10,
                                                 boost::bind(&ExtendedState_Data_t::feed, &fsm.extended_state_data_, _1));

    ros::Subscriber odom_sub =
        nh.subscribe<nav_msgs::Odometry>("odom",
                                         100,
                                         boost::bind(&Odom_Data_t::feed, &fsm.odom_data_, _1),
                                         ros::VoidConstPtr(),
                                         ros::TransportHints().tcpNoDelay());

    ros::Subscriber cmd_sub =
        nh.subscribe<quadrotor_msgs::PositionCommand>("cmd",
                                                      100,
                                                      boost::bind(&Command_Data_t::feed, &fsm.command_data_, _1),
                                                      ros::VoidConstPtr(),
                                                      ros::TransportHints().tcpNoDelay());

    ros::Subscriber imu_sub =
        nh.subscribe<sensor_msgs::Imu>("/mavros/imu/data", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                       100,
                                       boost::bind(&Imu_Data_t::feed, &fsm.imu_data_, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());
    ros::Subscriber emergency_sub = 
        nh.subscribe<std_msgs::Bool>("/emergency_landing",10, boost::bind(&Emergency_Landing_t::feed, &fsm.emergency_landing_, _1));

    ros::Subscriber rc_sub;
    if (!param.takeoff_land.no_RC) // mavros will still publish wrong rc messages although no RC is connected
    {
        rc_sub = nh.subscribe<mavros_msgs::RCIn>("/mavros/rc/in",
                                                 10,
                                                 boost::bind(&RC_Data_t::feed, &fsm.rc_data_, _1));
    }

    ros::Subscriber bat_sub =
        nh.subscribe<sensor_msgs::BatteryState>("/mavros/battery",
                                                100,
                                                boost::bind(&Battery_Data_t::feed, &fsm.battery_data_, _1),
                                                ros::VoidConstPtr(),
                                                ros::TransportHints().tcpNoDelay());

    ros::Subscriber takeoff_land_sub =
        nh.subscribe<quadrotor_msgs::TakeoffLand>("takeoff_land",
                                                  100,
                                                  boost::bind(&Takeoff_Land_Data_t::feed, &fsm.takeoff_land_data_, _1),
                                                  ros::VoidConstPtr(),
                                                  ros::TransportHints().tcpNoDelay());
    
    ros::Subscriber key_sub =
        nh.subscribe<std_msgs::String>("/keys", // Note: do NOT change it to /mavros/imu/data_raw !!!
                                       100,
                                       boost::bind(&Keyboard_t::feed, &fsm.keyboard_data_, _1),
                                       ros::VoidConstPtr(),
                                       ros::TransportHints().tcpNoDelay());
        

    fsm.ctrl_FCU_pub_ = nh.advertise<mavros_msgs::AttitudeTarget>("/mavros/setpoint_raw/attitude", 10);
    fsm.traj_start_trigger_pub_ = nh.advertise<geometry_msgs::PoseStamped>("/traj_start_trigger", 10);

    fsm.debug_pub_ = nh.advertise<quadrotor_msgs::Px4ctrlDebug>("/debugPx4ctrl", 10); // debug

    fsm.set_FCU_mode_srv_ = nh.serviceClient<mavros_msgs::SetMode>("/mavros/set_mode");
    fsm.arming_client_srv_ = nh.serviceClient<mavros_msgs::CommandBool>("/mavros/cmd/arming");
    fsm.reboot_FCU_srv_ = nh.serviceClient<mavros_msgs::CommandLong>("/mavros/cmd/command");

    ros::Duration(0.5).sleep();

    if (param.takeoff_land.no_RC)
    {
        ROS_WARN("PX4CTRL] Remote controller disabled, be careful!");
    }
    else
    {
        ROS_INFO("PX4CTRL] Waiting for RC");
        while (ros::ok())
        {
            ros::spinOnce();
            if (fsm.rc_is_received(ros::Time::now()))
            {
                ROS_INFO("[PX4CTRL] RC received.");
                break;
            }
            ros::Duration(0.1).sleep();
        }
    }

    int trials = 0;
    while (ros::ok() && !fsm.state_data_.current_state.connected)
    {
        ros::spinOnce();
        ros::Duration(1.0).sleep();
        if (trials++ > 5)
            ROS_ERROR("Unable to connnect to PX4!!!");
    }

    ros::Rate r(param.ctrl_freq_max);
    while (ros::ok())
    {
        r.sleep();
        ros::spinOnce();
        fsm.process(); // We DO NOT rely on feedback as trigger, since there is no significant performance difference through our test.
    }

    return 0;
}
