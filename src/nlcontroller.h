/*
 * @Author: xindong324
 * @Date: 2022-07-02 22:34:00
 * @LastEditors: xindong324
 * @LastEditTime: 2022-09-23 17:18:18
 * @Description: file content
 */
/*************************************************************/
/* Acknowledgement: github.com/uzh-rpg/rpg_quadrotor_control */
/*************************************************************/

#ifndef __NLCONTROLLER_H
#define __NLCONTROLLER_H

#include <mavros_msgs/AttitudeTarget.h>
#include <quadrotor_msgs/Px4ctrlDebug.h>
#include <queue>

#include "input.h"
#include <Eigen/Dense>

#include <my_px4ctrl/converters.h>
#include <my_px4ctrl/utils.h>
#include <my_px4ctrl/geometry_utils.h>



// double quaternion2yaw(Eigen::Quaterniond q)
// {
//   double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
//   return yaw;
// }

// struct Desired_State_t
// {
// 	Eigen::Vector3d p;
// 	Eigen::Vector3d v;
// 	double yaw;
// 	double head_rate;
// 	Eigen::Quaterniond q;
// 	Eigen::Vector3d a;
// 	Eigen::Vector3d jerk;
// };

// struct Controller_Output_t
// {
//     static constexpr double CTRL_YAW_RATE = 1.0;
//     static constexpr double CTRL_YAW = 0.0;

// 	double roll;
// 	double pitch;
// 	double yaw;
// 	double thrust;
// 	double roll_rate;
// 	double pitch_rate;
// 	double yaw_rate;
// 	double yaw_mode; // if yaw_mode > 0, CTRL_YAW;
// 				// if yaw_mode < 0, CTRL_YAW_RATE
// 	Eigen::Quaterniond orientation;
// 	double normalized_thrust;

// 	Eigen::Vector3d des_v_real;
// };

struct SO3_Controller_Output_t
{
	Eigen::Matrix3d Rdes;
	Eigen::Vector3d Fdes;
	double net_force;
};

struct Desired_State_t
{
	Eigen::Vector3d p;
	Eigen::Vector3d v;
	Eigen::Vector3d a;
	Eigen::Vector3d j;
	Eigen::Quaterniond q;
	double yaw;
	double yaw_rate;

	Desired_State_t(){};

	Desired_State_t(Odom_Data_t &odom)
		: p(odom.p),
		  v(Eigen::Vector3d::Zero()),
		  a(Eigen::Vector3d::Zero()),
		  j(Eigen::Vector3d::Zero()),
		  q(odom.q),
		  yaw(myuav_utils::get_yaw_from_quaternion(odom.q)),
		  yaw_rate(0){};
};

struct Controller_Output_t
{

	// Orientation of the body frame with respect to the world frame
	Eigen::Quaterniond q;

	// Body rates in body frame
	Eigen::Vector3d bodyrates; // [rad/s]

  double normalized_thrust;

	// Collective mass normalized thrust
	double thrust;

	//Eigen::Vector3d des_v_real;
};


class Controller
{
public:
  Controller(Parameter_t &);

  Eigen::Matrix3d Kp;
  Eigen::Matrix3d Kv;
  Eigen::Matrix3d Kvi;
  Eigen::Matrix3d Ka;
  double KAngR, KAngP, KAngY;

  void config_gain(const Parameter_t::Gain& gain);
  void config();

  Eigen::Vector3d int_e_v;

  quadrotor_msgs::Px4ctrlDebug calculateControl(const Desired_State_t &des,
      const Odom_Data_t &odom,
      const Imu_Data_t &imu, 
      Controller_Output_t &u);
  
  Controller_Output_t computeNominalReferenceInputs(
    const Desired_State_t& reference_state,
    const Odom_Data_t& attitude_estimate) const;
  Eigen::Vector3d computePIDErrorAcc(
    const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu) const;

  Eigen::Vector3d computePIDErrorAccZJU(
    const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu);

  Eigen::Quaterniond computeDesiredAttitude(
    const Eigen::Vector3d& desired_acceleration, const double reference_heading,
    const Eigen::Quaterniond& attitude_estimate) const;
	bool almostZero(const double value) const;
	bool almostZeroThrust(const double thrust_value) const;
  Eigen::Vector3d computeRobustBodyXAxis(
    const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
    const Eigen::Vector3d& y_C,
    const Eigen::Quaterniond& attitude_estimate) const; 
  Eigen::Vector3d computeFeedBackControlBodyrates(const Eigen::Quaterniond& desired_attitude,
    const Eigen::Quaterniond& attitude_estimate);
  // Eigen::Vector3d computePIDErrorAcc(
  //   const Odom_Data_t& state_estimate,
  //   const Desired_State_t& reference_state);
  // Eigen::Vector3d computePIDErrorAccZJU(
  //   const Odom_Data_t& state_estimate,
  //   const Desired_State_t& reference_state);

  void publish_ctrl(const Controller_Output_t& u, const ros::Time& stamp);
  void publish_zero_ctrl(const ros::Time& stamp);
  // set controller
  bool estimateThrustModel(const Eigen::Vector3d &est_v,
      const Parameter_t &param);
  void resetThrustMapping(void);

  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

private:
  Parameter_t param_;
  quadrotor_msgs::Px4ctrlDebug debug_msg_;
  std::queue<std::pair<ros::Time, double>> timed_thrust_;
  static constexpr double kMinNormalizedCollectiveThrust_ = 3.0;
  bool is_configured;
  // Thrust-accel mapping params
  const double rho2_ = 0.998; // do not change
  double thr2acc_;
  double P_;

  double computeDesiredCollectiveThrustSignal(const Eigen::Vector3d &des_acc);
  //double fromQuaternion2yaw(Eigen::Quaterniond q);
};


#endif
