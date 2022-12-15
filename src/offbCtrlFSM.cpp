#include "offbCtrlFSM.h"
#include <my_px4ctrl/converters.h>
#include <my_px4ctrl/utils.h>

using namespace std;
using namespace myuav_utils;

OffbCtrlFSM::OffbCtrlFSM(Parameter_t &param, Controller &controller): param_(param), controller_(controller)
{
    state_ = INIT;
    hover_pose_.setZero();
}

// the rc should be switch to auto hover  then can switch to hover mode.
// and sw to command mode when using auto takeoff mannual and auto land



/* 
        Finite State Machine

	      system start
	            |
	            |
	            v
	----- > MANUAL_CTRL <-----------------
	|         ^   |    \                 |
	|         |   |     \                |
	|         |   |      > AUTO_TAKEOFF  |
	|         |   |        /             |
	|         |   |       /              |
	|         |   |      /               |
	|         |   v     /                |
	|       AUTO_HOVER <                 |
	|         ^   |  \  \                |
	|         |   |   \  \               |
	|         |	  |    > AUTO_LAND -------
	|         |   |
	|         |   v
	-------- CMD_CTRL

*/

void OffbCtrlFSM::process()
{
    ros::Time now_time = ros::Time::now();
    Controller_Output_t u;
    Desired_State_t des(odom_data_);
    bool rotor_low_speed_during_land = false;

    if(emergency_landing_.flag_emergency_landing) 
        state_ = EMERGENCY_STOP;

    // STEP1: state machine runs
    switch (state_)
    {
    case INIT:
    {    /* code */
        /********************** check state change **************************/
        
        controller_.resetThrustMapping();
        if(param_.takeoff_land.enable && keyboard_data_.trigger_)
        {// if takeoff command triggered // Try to jump to AUTO_TAKEOFF
            if(!odom_is_received(now_time))
            {
                ROS_ERROR("[px4ctrl] Reject Auto_takeoff. No odom.");
                break;
            }
            if(cmd_is_received(now_time))
            {
                ROS_ERROR("[px4ctrlfsm] Reject auto takeoff. you are sending commands before toggling into auto takeoff");
                break;
            }
            if(odom_data_.v.norm() > 0.1)
            {
                ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. Odom_Vel=%fm/s, non-static takeoff is not allowed!", odom_data_.v.norm());
                break;
            }
            if(!get_landed())
            {
                ROS_ERROR("[px4ctrl] Reject AUTO_TAKEOFF. land detector says that the drone is not landed now.");
                break;
            }
            
            state_ = AUTO_TAKEOFF;
            controller_.resetThrustMapping();
            set_start_pose_for_takeoff_land(odom_data_);
            toggle_offboard_mode(true); // change to offboard before arm
            // wait for 0.1 sec
            for(int i = 0; i < 10 && ros::ok(); i++)
            {
                ros::Duration(0.01).sleep();
                ros::spinOnce();
            }
            if(param_.takeoff_land.enable_auto_arm)
            {
                toggle_arm_disarm(true);
            }
            takeoff_land_.toggle_takeoff_land_time = now_time;
            ROS_INFO("\033[32m[px4ctrl] MANUAL_CTRL(L1) --> AUTO_TAKEOFF\033[32m");
        }

        
        break;
    }

    // if no odom or not in hover by rc, change to mannual
    // if in cmd mode rc & cmd rcv.
    case AUTO_HOVER:
    {
        
        if( cmd_is_received(now_time))
        {
            state_ = CMD_CTRL;
            des = get_cmd_des();
            ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> CMD_CTRL(L3)\033[32m");
        }
        else if(keyboard_data_.land_trigger_)
        {
            state_ = AUTO_LAND;
            set_start_pose_for_takeoff_land(odom_data_);
            ROS_INFO("\033[32m[px4ctrl] AUTO_HOVER(L2) --> AUTO_LAND(L3)\033[32m");
        }
        else{
            //normal hover mission
            set_hov_with_rc();
            des = get_hover_des();

            if(rc_data_.enter_command_mode || 
                (takeoff_land_.delay_trigger.first && now_time > takeoff_land_.delay_trigger.second))
                {
                    takeoff_land_.delay_trigger.first = false;
                    publish_trigger(odom_data_.msg);
                    ROS_INFO("\033[32m[px4ctrl] TRIGGER sent, allow user command.\033[32m");
                }
        }
        break;
    }

    case CMD_CTRL:
    {
        if(!cmd_is_received(now_time))
        {
            state_ = AUTO_HOVER;
            set_hov_with_odom();
            des = get_hover_des();
            ROS_INFO("[px4ctrl] From CMD_CTRL(L3) to AUTO_HOVER(L2)!");
        }
        else{
            des = get_cmd_des();
        }
        if(keyboard_data_.land_trigger_)
        {
            state_ = AUTO_LAND;
            ROS_INFO("land");
        }
        break;
    }

    case AUTO_TAKEOFF:
    {
        if((now_time - takeoff_land_.toggle_takeoff_land_time).toSec() < AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME)
        {
            des = get_rotor_speed_up_des(now_time);
        }else if(odom_data_.p(2) >= (takeoff_land_.start_pose(2) + param_.takeoff_land.height))
        {// reach the desired height
            state_ = AUTO_HOVER;
            set_hov_with_odom();
            ROS_INFO("\033[32m[px4ctrl] AUTO_TAKEOFF --> AUTO_HOVER(L2)\033[32m");

            takeoff_land_.delay_trigger.first = true;
            takeoff_land_.delay_trigger.second = now_time + ros::Duration(AutoTakeoffLand_t::DELAY_TRIGGER_TIME);
        }
        else{
            des = get_takeoff_land_des(param_.takeoff_land.speed);
        }
        break;
    }

    case AUTO_LAND:
    {
        if(!get_landed())
        {
            des = get_takeoff_land_des(-param_.takeoff_land.speed);
        }
        else{
            rotor_low_speed_during_land = true;
            static bool print_once_flag = true;
            if(print_once_flag)
            {
                ROS_INFO("\033[32m[px4ctrl] Wait for abount 10s to let the drone arm.\033[32m");
                print_once_flag = false;
            }

            if(extended_state_data_.current_extended_state.landed_state = mavros_msgs::ExtendedState::LANDED_STATE_ON_GROUND)
            {// px4 allow arm
                static double last_trial_time = 0;
                if(now_time.toSec() - last_trial_time > 1.0)
                {
                    if(toggle_arm_disarm(false))
                    {
                        print_once_flag = true;
                        keyboard_data_.land_trigger_ = false;
                        state_ = INIT;
                        toggle_offboard_mode(false);
                        ROS_INFO("\033[32m[px4ctrl] AUTO_LAND --> MANUAL_CTRL(L1)\033[32m");
                    }
                    last_trial_time = now_time.toSec();
                } 
            }
        }
        break;
    }
    
    case EMERGENCY_STOP:
    {
        u.thrust = 0;
        emergency_landing_.flag_emergency_landing = true;
        break;
    }
    
    default:
        break;
    }

    //cout<<"imu data: " << imu_data_.a(0) << imu_data_.a(1) << imu_data_.a(2) << endl;
    //cout<<"odom data: "<< odom_data_.p(0) << odom_data_.p(1) << odom_data_.p(2) << endl;
    //step2: estimate thrust model
    if(state_ == AUTO_HOVER || state_ == CMD_CTRL)
    {
        controller_.estimateThrustModel(imu_data_.a, param_);
    }

    
    // step3: solve and update control commands
    if(rotor_low_speed_during_land) // used at start of auto takeoff
    {
        motors_idling(imu_data_, u);
    }
    else{
        debug_msg_ = controller_.calculateControl(des, odom_data_, imu_data_, u);
        // cout << "thrust" << u.thrust << endl;
        debug_msg_.header.stamp = now_time;
        debug_pub_.publish(debug_msg_);
    }

    // STEP 2.5: check emergency
    if(state_ == EMERGENCY_STOP || emergency_landing_.flag_emergency_landing)
    {
        //u.q = Eigen::Quaterniond();
        u.thrust = 0;
        debug_msg_.des_thr = 0;
    }
    // step4: publish control commands to mavros
    if(param_.use_bodyrate_ctrl)
    {
        publish_bodyrate_ctrl(u, now_time);
    }
    else{
        publish_attitude_ctrl(u, now_time);
    }

    //step5: Detect if the drone has landed
    land_detector(state_, des, odom_data_);

    // step6 clear flags beyound their lifetime
    rc_data_.enter_hover_mode = false;
    rc_data_.enter_command_mode = false;
    takeoff_land_data_.triggered = false;
}

void OffbCtrlFSM::motors_idling(const Imu_Data_t &imu, Controller_Output_t &u)
{
    u.q = imu.q;
    u.bodyrates = Eigen::Vector3d::Zero();
    u.thrust = 0.04;
}

void OffbCtrlFSM::land_detector(const State_t state, const Desired_State_t &des, const Odom_Data_t &odom)
{
    static State_t last_state = State_t::INIT;
    if(last_state == State_t::INIT && (state == State_t::AUTO_HOVER || state == State_t::AUTO_TAKEOFF))
    {// if not landing mode, always hold false
        takeoff_land_.landed = false;
    }

    last_state = state;

    if(state == State_t::INIT && !state_data_.current_state.armed)
    {// no need futher decisions
        takeoff_land_.landed = true;
        return;
    }

    // land detector param
    constexpr double POSITION_DEVIATION_C = -0.5;   // constrain 1: target position below real position for POSITION_DEVIATION_C meters
    constexpr double VELOCITY_THR_C = 0.1;          // constrain 2: velocity below velocity_min_c m/s
    constexpr double TIME_KEEP_C = 3.0;             // constrain 3: time the constrain 1&2 need to keep

    static ros::Time time_C12_reached; // time constrain 12 reached
    static bool is_last_C12_satisfy = false;

    if(takeoff_land_.landed)
    {
        time_C12_reached = ros::Time::now();
        is_last_C12_satisfy = false;
    }
    else{
        bool C12_satisfy = (des.p(2) - odom.p(2)) < POSITION_DEVIATION_C && odom.v.norm() < VELOCITY_THR_C;
        if(C12_satisfy && !is_last_C12_satisfy)
        {
            time_C12_reached = ros::Time::now();
        }
        else if(C12_satisfy && is_last_C12_satisfy)
        {
            if((ros::Time::now() - time_C12_reached).toSec() > TIME_KEEP_C) // constrain 3 reached
            {
                takeoff_land_.landed = true;
            }
        }
        is_last_C12_satisfy = C12_satisfy;
    }
}

Desired_State_t OffbCtrlFSM::get_hover_des()
{
	Desired_State_t des;
	des.p = hover_pose_.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = hover_pose_(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t OffbCtrlFSM::get_cmd_des()
{
	Desired_State_t des;
	des.p = command_data_.p;
	des.v = command_data_.v;
	des.a = command_data_.a;
	des.j = command_data_.j;
	des.yaw = command_data_.yaw;
	des.yaw_rate = command_data_.yaw_rate;

	return des;
}

Desired_State_t OffbCtrlFSM::get_rotor_speed_up_des(const ros::Time &now)
{
	double delta_t = (now - takeoff_land_.toggle_takeoff_land_time).toSec();
	double des_a_z = exp((delta_t - AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME) * 6.0) * 7.0 - 7.0; // Parameters 6.0 and 7.0 are just heuristic values which result in a saticfactory curve.
	if (des_a_z > 0.1)
	{
		ROS_ERROR("des_a_z > 0.1!, des_a_z=%f", des_a_z);
		des_a_z = 0.0;
	}

	Desired_State_t des;
	des.p = takeoff_land_.start_pose.head<3>();
	des.v = Eigen::Vector3d::Zero();
	des.a = Eigen::Vector3d(0, 0, des_a_z);
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land_.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

Desired_State_t OffbCtrlFSM::get_takeoff_land_des(const double speed)
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - takeoff_land_.toggle_takeoff_land_time).toSec() - (speed > 0 ? AutoTakeoffLand_t::MOTORS_SPEEDUP_TIME : 0); // speed > 0 means takeoff
	// takeoff_land.last_set_cmd_time = now;

	// takeoff_land.start_pose(2) += speed * delta_t;

	Desired_State_t des;
	des.p = takeoff_land_.start_pose.head<3>() + Eigen::Vector3d(0, 0, speed * delta_t);
	des.v = Eigen::Vector3d(0, 0, speed);
	des.a = Eigen::Vector3d::Zero();
	des.j = Eigen::Vector3d::Zero();
	des.yaw = takeoff_land_.start_pose(3);
	des.yaw_rate = 0.0;

	return des;
}

void OffbCtrlFSM::set_hov_with_odom()
{
	hover_pose_.head<3>() = odom_data_.p;
	hover_pose_(3) = get_yaw_from_quaternion(odom_data_.q);

	last_set_hover_pose_time_ = ros::Time::now();
}

void OffbCtrlFSM::set_hov_with_rc()
{
	ros::Time now = ros::Time::now();
	double delta_t = (now - last_set_hover_pose_time_).toSec();
	last_set_hover_pose_time_ = now;

	hover_pose_(0) += rc_data_.ch[1] * param_.max_manual_vel * delta_t * (param_.rc_reverse.pitch ? 1 : -1);
	hover_pose_(1) += rc_data_.ch[0] * param_.max_manual_vel * delta_t * (param_.rc_reverse.roll ? 1 : -1);
	hover_pose_(2) += rc_data_.ch[2] * param_.max_manual_vel * delta_t * (param_.rc_reverse.throttle ? 1 : -1);
	hover_pose_(3) += rc_data_.ch[3] * param_.max_manual_vel * delta_t * (param_.rc_reverse.yaw ? 1 : -1);

	if (hover_pose_(2) < -0.3)
		hover_pose_(2) = -0.3;

	// if (param.print_dbg)
	// {
	// 	static unsigned int count = 0;
	// 	if (count++ % 100 == 0)
	// 	{
	// 		cout << "hover_pose=" << hover_pose.transpose() << endl;
	// 		cout << "ch[0~3]=" << rc_data.ch[0] << " " << rc_data.ch[1] << " " << rc_data.ch[2] << " " << rc_data.ch[3] << endl;
	// 	}
	// }
}

void OffbCtrlFSM::set_start_pose_for_takeoff_land(const Odom_Data_t &odom)
{
	takeoff_land_.start_pose.head<3>() = odom_data_.p;
	takeoff_land_.start_pose(3) = get_yaw_from_quaternion(odom_data_.q);

	takeoff_land_.toggle_takeoff_land_time = ros::Time::now();
}

bool OffbCtrlFSM::rc_is_received(const ros::Time &now_time)
{
	return (now_time - rc_data_.rcv_stamp).toSec() < param_.msg_timeout.rc;
}

bool OffbCtrlFSM::cmd_is_received(const ros::Time &now_time)
{
	return (now_time - command_data_.rcv_stamp).toSec() < param_.msg_timeout.cmd;
}

bool OffbCtrlFSM::odom_is_received(const ros::Time &now_time)
{
	return (now_time - odom_data_.rcv_stamp).toSec() < param_.msg_timeout.odom;
}

bool OffbCtrlFSM::imu_is_received(const ros::Time &now_time)
{
	return (now_time - imu_data_.rcv_stamp).toSec() < param_.msg_timeout.imu;
}

bool OffbCtrlFSM::bat_is_received(const ros::Time &now_time)
{
	return (now_time - battery_data_.rcv_stamp).toSec() < param_.msg_timeout.bat;
}

bool OffbCtrlFSM::recv_new_odom()
{
	if (odom_data_.recv_new_msg)
	{
		odom_data_.recv_new_msg = false;
		return true;
	}

	return false;
}

void OffbCtrlFSM::publish_bodyrate_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ATTITUDE;

	msg.body_rate.x = u.bodyrates.x();
	msg.body_rate.y = u.bodyrates.y();
	msg.body_rate.z = u.bodyrates.z();

	msg.thrust = u.thrust;
    
    
	ctrl_FCU_pub_.publish(msg);
}

void OffbCtrlFSM::publish_attitude_ctrl(const Controller_Output_t &u, const ros::Time &stamp)
{
	mavros_msgs::AttitudeTarget msg;

	msg.header.stamp = stamp;
	msg.header.frame_id = std::string("FCU");

	// msg.type_mask = mavros_msgs::AttitudeTarget::IGNORE_ROLL_RATE |
	// 				mavros_msgs::AttitudeTarget::IGNORE_PITCH_RATE |
	// 				mavros_msgs::AttitudeTarget::IGNORE_YAW_RATE;

	msg.orientation.x = u.q.x();
	msg.orientation.y = u.q.y();
	msg.orientation.z = u.q.z();
	msg.orientation.w = u.q.w();

    msg.body_rate.x = u.bodyrates.x();
    msg.body_rate.y = u.bodyrates.y();
    msg.body_rate.z = u.bodyrates.z();

	msg.thrust = u.thrust;

	ctrl_FCU_pub_.publish(msg);
}

void OffbCtrlFSM::publish_trigger(const nav_msgs::Odometry &odom_msg)
{
	geometry_msgs::PoseStamped msg;
	msg.header.frame_id = "world";
	msg.pose = odom_msg.pose.pose;

	traj_start_trigger_pub_.publish(msg);
}

bool OffbCtrlFSM::toggle_offboard_mode(bool on_off)
{
	mavros_msgs::SetMode offb_set_mode;

	if (on_off)
	{
		state_data_.state_before_offboard = state_data_.current_state;
		if (state_data_.state_before_offboard.mode == "OFFBOARD") // Not allowed
			state_data_.state_before_offboard.mode = "MANUAL";

		offb_set_mode.request.custom_mode = "OFFBOARD";
		if (!(set_FCU_mode_srv_.call(offb_set_mode) && offb_set_mode.response.mode_sent))
		{
			ROS_ERROR("Enter OFFBOARD rejected by PX4!");
			return false;
		}
	}
	else
	{
		offb_set_mode.request.custom_mode = state_data_.state_before_offboard.mode;
		if (!(set_FCU_mode_srv_.call(offb_set_mode) && offb_set_mode.response.mode_sent))
		{
			ROS_ERROR("Exit OFFBOARD rejected by PX4!");
			return false;
		}
	}

	return true;

	// if (param.print_dbg)
	// 	printf("offb_set_mode mode_sent=%d(uint8_t)\n", offb_set_mode.response.mode_sent);
}

bool OffbCtrlFSM::toggle_arm_disarm(bool arm)
{
	mavros_msgs::CommandBool arm_cmd;
	arm_cmd.request.value = arm;
	if (!(arming_client_srv_.call(arm_cmd) && arm_cmd.response.success))
	{
		if (arm)
			ROS_ERROR("ARM rejected by PX4!");
		else
			ROS_ERROR("DISARM rejected by PX4!");

		return false;
	}

	return true;
}

void OffbCtrlFSM::reboot_FCU()
{
	// https://mavlink.io/en/messages/common.html, MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN(#246)
	mavros_msgs::CommandLong reboot_srv;
	reboot_srv.request.broadcast = false;
	reboot_srv.request.command = 246; // MAV_CMD_PREFLIGHT_REBOOT_SHUTDOWN
	reboot_srv.request.param1 = 1;	  // Reboot autopilot
	reboot_srv.request.param2 = 0;	  // Do nothing for onboard computer
	reboot_srv.request.confirmation = true;

	reboot_FCU_srv_.call(reboot_srv);

	ROS_INFO("Reboot FCU");

	// if (param.print_dbg)
	// 	printf("reboot result=%d(uint8_t), success=%d(uint8_t)\n", reboot_srv.response.result, reboot_srv.response.success);
}
