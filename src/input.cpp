#include "input.h"
#include <my_px4ctrl/utils.h>

using namespace std;

RC_Data_t::RC_Data_t()
{
    rcv_stamp = ros::Time(0);
    
    last_mode = -1.0;
    last_gear = -1.0;

    // Parameter init is very important in rc-free
    is_hover_mode = true;
    enter_hover_mode = false;
    is_command_mode = true;
    enter_command_mode = false;
    toggle_reboot = false;
    for(int i = 0; i < 4; ++i)
    {
        ch[i] = 0.0;
    }
}



void RC_Data_t::feed(mavros_msgs::RCInConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    for(int i = 0; i < 4; i++)
    {
        ch[i] = ((double)msg.channels[i] - 1500.0)/500.0;
        if(ch[i] > DEAD_ZONE)
            ch[i] = (ch[i] - DEAD_ZONE) / (1-DEAD_ZONE);
        if(ch[i] < -DEAD_ZONE)
            ch[i] = (ch[i] + DEAD_ZONE) / (1 - DEAD_ZONE);
        else ch[i] = 0.0;
    }

    mode = ((double)msg.channels[4] - 1000.0) / 1000.0;
    gear = ((double)msg.channels[5] - 1000.0) / 1000.0;
    reboot_cmd = ((double)msg.channels[7] - 1000.0) / 1000.0;

    check_validity();

    if(!have_init_last_mode)
    {
        have_init_last_mode = true;
        last_mode = mode;
    }
    if(!have_init_last_gear)
    {
        have_init_last_gear = true;
        last_gear = gear;
    }
    if(!have_init_last_reboot_cmd)
    {
        have_init_last_reboot_cmd = true;
        last_reboot_cmd = reboot_cmd;
    }

    // judge if enter hover mode
    if(last_mode < API_MODE_THRESHOLD_VALUE && mode > API_MODE_THRESHOLD_VALUE)
        enter_hover_mode = true;
    else 
        enter_hover_mode = false;
    
    if(mode > API_MODE_THRESHOLD_VALUE)
        is_hover_mode = true;
    else 
        is_hover_mode = false;
    
    // 2 
    if(is_hover_mode)
    {
        if(last_gear < GEAR_SHIFT_VALUE && gear > GEAR_SHIFT_VALUE)
            enter_command_mode = true;
        else if(gear < GEAR_SHIFT_VALUE)
            enter_command_mode = false;
        
        if(gear > GEAR_SHIFT_VALUE)
            is_command_mode = true;
        else 
            is_command_mode = false;
    }

    //3 
    if(!is_hover_mode && !is_command_mode)
    {
        if(last_reboot_cmd < REBOOT_THRESHOLD_VALUE && reboot_cmd > REBOOT_THRESHOLD_VALUE)
            toggle_reboot = true;
        else 
            toggle_reboot = false;
    }
    else 
        toggle_reboot = false;
    
    last_mode = mode;
    last_gear = gear;
    last_reboot_cmd = reboot_cmd;
}

/**
 * @brief check if ch[4]-mode & ch[5]-gear & ch[7]-reboot in reasonable range [-1.1, 1.1]
 * 
 */
void RC_Data_t::check_validity()
{
    if(mode >= -1.1 && mode <= 1.1 && gear >= -1.1 && gear <= 1.1 && reboot_cmd >= -1.1 && reboot_cmd <= 1.1)
    {
        // pass
    }
    else{
        ROS_ERROR("RC data validity check fail. mode=%f, gear= %f, reboot_cmd=%f", mode, gear, reboot_cmd);
    }
}

/**
 * @brief check if ch[0-3] all in center
 * 
 * @return true  all sticks of remotor in center
 * @return false at least one of sticks not in center
 */
bool RC_Data_t::check_centered()
{
    bool centered = abs(ch[0]) < 1e-5 && abs(ch[1]) < 1e-5 && abs(ch[2]) < 1e-5 && abs(ch[3]) < 1e-5 ;
    return centered;
}

Odom_Data_t::Odom_Data_t()
{
    rcv_stamp = ros::Time(0);
    q.setIdentity();
    recv_new_msg = false;
}

/**
 * @brief feed odom data to p,v,q,w, recommened to 100Hz
 * 
 * @param pMsg odom msg from fc
 */
void Odom_Data_t::feed(nav_msgs::OdometryConstPtr pMsg)
{
    ros::Time now = ros::Time::now();

    msg = *pMsg;
    rcv_stamp = now;
    recv_new_msg = true;
    myuav_utils::extract_odometry(pMsg, p, v, q, w);

    // #define VEL_IN_BODY
    // set to 1 if the velocity in odom topic is relative to current body frame, not to world frame
    #ifdef VEL_IN_BODY
        Eigen::Quaterniond wRb_q(msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z);
        Eigen::Matrix3d wRb = wRb_q.matrix();
        v = wRb * v;    // trans into world
        
        static int count = 0;
        if(count++ % 500 == 0)
            ROS_WARN("VEL IN BODY!");

    #endif

    // check freq
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0);
    if((now - last_clear_count_time).toSec() > 1.0)
    {
        if(one_min_count < 100)
        {
            ROS_WARN("ODOM frequency seems lower than 100Hz, which is too low!");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count++;
}

Imu_Data_t::Imu_Data_t()
{
    rcv_stamp = ros::Time(0);
}

/**
 * @brief feed imu data into class member
 * 
 * 
 * @param pMsg imu data
 */
void Imu_Data_t::feed(sensor_msgs::ImuConstPtr pMsg)
{
    ros::Time now = ros::Time::now();
    msg = *pMsg;
    rcv_stamp = now;

    w(0) = msg.angular_velocity.x;
    w(1) = msg.angular_velocity.y;
    w(2) = msg.angular_velocity.z;

    a(0) = msg.linear_acceleration.x;
    a(1) = msg.linear_acceleration.y;
    a(2) = msg.linear_acceleration.z;

    q.x() = msg.orientation.x;
    q.y() = msg.orientation.y;
    q.z() = msg.orientation.z;
    q.w() = msg.orientation.w;

    //check the freq
    static int one_min_count = 9999;
    static ros::Time last_clear_count_time = ros::Time(0.0);
    if((now - last_clear_count_time).toSec() > 1.0)
    {
        if(one_min_count < 100)
        {
            ROS_WARN("IMU freq seems lower than 100 Hz, which is too low!");
        }
        one_min_count = 0;
        last_clear_count_time = now;
    }
    one_min_count++;
}

State_Data_t::State_Data_t()
{

}

void State_Data_t::feed(mavros_msgs::StateConstPtr pMsg)
{
    current_state = *pMsg;
}

ExtendedState_Data_t::ExtendedState_Data_t()
{
    
}

void ExtendedState_Data_t::feed(mavros_msgs::ExtendedStateConstPtr pMsg)
{
    current_extended_state = *pMsg;
}

Command_Data_t::Command_Data_t()
{
    rcv_stamp = ros::Time(0);
    cmd_init = false;
}

void Command_Data_t::feed(quadrotor_msgs::PositionCommandConstPtr pMsg)
{
    msg = *pMsg;
    ros::Time now = ros::Time::now();

    if((rcv_stamp - now).toSec() > 0.5)
    {
        cmd_init = false;
        cout << "rcv cmd break" << endl;
    }
    else
        cmd_init = true;

    rcv_stamp = now;

    //cout << "rcv cmd" << endl;

    p(0) = msg.position.x;
    p(1) = msg.position.y;
    p(2) = msg.position.z;

    v(0) = msg.velocity.x;
    v(1) = msg.velocity.y;
    v(2) = msg.velocity.z;

    a(0) = msg.acceleration.x;
    a(1) = msg.acceleration.y;
    a(2) = msg.acceleration.z;

    j(0) = msg.jerk.x;
    j(1) = msg.jerk.y;
    j(2) = msg.jerk.z;

    yaw = myuav_utils::normalize_angle(msg.yaw);
    yaw_rate = msg.yaw_dot;
}

Battery_Data_t::Battery_Data_t()
{
    rcv_stamp = ros::Time(0.0);

}

void Battery_Data_t::feed(sensor_msgs::BatteryStateConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    double voltage = 0;
    for (size_t i = 0; i < msg.cell_voltage.size(); i++)
    {
        /* code */
        voltage += pMsg->cell_voltage[i];
    }
    volt = 0.8 * volt + 0.2 * voltage;  //LPF

    percentage = msg.percentage;

    static ros::Time last_print_t = ros::Time(0);

    if(percentage > 0.05)
    {
        if((rcv_stamp - last_print_t).toSec() > 10)
        {
            ROS_INFO("[px4ctrl] voltage=%.3f, percentage=%.3f", volt, percentage);
            last_print_t = rcv_stamp;
        }
    }else{
        if((rcv_stamp - last_print_t).toSec() > 1)
        {
            last_print_t = rcv_stamp;
        }
    }
    
}

Takeoff_Land_Data_t::Takeoff_Land_Data_t()
{
    rcv_stamp = ros::Time(0.0);
}

void Takeoff_Land_Data_t::feed(quadrotor_msgs::TakeoffLandConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    triggered = true;
    takeoff_land_cmd = pMsg->takeoff_land_cmd;
}

Keyboard_t::Keyboard_t()
{
    rcv_stamp = ros::Time(0.0);
    trigger_ = false;
    land_trigger_ = false;
}

void Keyboard_t::feed(std_msgs::StringConstPtr str)
{
    if(str->data == "d" || str->data == "D")
    {
        trigger_ = false;
        land_trigger_ = true;
    }
    else if(str->data == "g" || str->data == "G")
    {
        trigger_ = false;
    }
    else if(str->data == "a" || str->data == "A")
    {// take off
        trigger_ = true;
    }
    else if(str->data == "w" || str->data == "W")
    {
        start_mission_ = true;
    }
    else if(str->data == "j" || str->data == "J")
    {
        image_yaw_state_ = 1;
    }
    else if(str->data == "k" || str->data == "K")
    {
        image_yaw_state_ = 0;
    }
    std::cout<<"str.data : "<< str->data <<std::endl;
}

Emergency_Landing_t::Emergency_Landing_t()
{
    flag_emergency_landing = false;
    rcv_stamp = ros::Time(0);
}

void Emergency_Landing_t::feed(std_msgs::BoolConstPtr pMsg)
{
    rcv_stamp = ros::Time::now();
    flag_emergency_landing = pMsg->data;
}