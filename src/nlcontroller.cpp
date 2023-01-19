#include "nlcontroller.h"
#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <geometry_msgs/Vector3Stamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <boost/format.hpp>
#include "std_msgs/Float32.h"

using namespace std;
using namespace myuav_utils;


// double Controller::fromQuaternion2yaw(Eigen::Quaterniond q)
// {
//   double yaw = atan2(2 * (q.x()*q.y() + q.w()*q.z()), q.w()*q.w() + q.x()*q.x() - q.y()*q.y() - q.z()*q.z());
//   return yaw;
// }

Controller::Controller(Parameter_t &param) : param_(param)
{
  is_configured = false;
	int_e_v.setZero();
  //resetThrustMapping();
}

// void Controller::config()
// {
// 	config_gain(param_.gain);
// 	is_configured = true;
// }

Controller_Output_t Controller::computeNominalReferenceInputs(
  const Desired_State_t& referance_state,
  const Odom_Data_t& attitude_estimate) const{

    Controller_Output_t referance_command;
    const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
        Eigen::AngleAxisd(referance_state.yaw,Eigen::Vector3d::UnitZ()));
    
    const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();
    const Eigen::Vector3d des_acc = referance_state.a + Eigen::Vector3d(0,0,param_.gra);
    // Reference attitude
    const Eigen::Quaterniond q_W_B = computeDesiredAttitude(
      des_acc, referance_state.yaw, attitude_estimate.q);
    
    const Eigen::Vector3d x_B = q_W_B * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d y_B = q_W_B * Eigen::Vector3d::UnitY();
    const Eigen::Vector3d z_B = q_W_B * Eigen::Vector3d::UnitZ();
  
    referance_command.q = q_W_B;

    //reference thrust
    referance_command.normalized_thrust = des_acc.norm();
    
    //referance body
    if(almostZeroThrust(referance_command.normalized_thrust))
    {
      referance_command.bodyrates.x() = 0;
      referance_command.bodyrates.y() = 0;
    }
    else{
      referance_command.bodyrates.x() = -1.0/referance_command.normalized_thrust * y_B.dot(referance_state.j);
      referance_command.bodyrates.y() = 1.0/referance_command.normalized_thrust * x_B.dot(referance_state.j);
    }

    if(almostZero((y_C.cross(z_B)).norm())){
      referance_command.bodyrates.z() = 0.0;
    }else{
      referance_command.bodyrates.z() = 
        1.0/(y_C.cross(z_B)).norm() * 
        (referance_state.yaw_rate * x_C.dot(x_B)+
         referance_command.bodyrates.y() * y_C.dot(z_B));
    }
    return referance_command;
  }

Eigen::Quaterniond Controller::computeDesiredAttitude(
  const Eigen::Vector3d& desired_acceleration, const double referance_heading,
  const Eigen::Quaterniond& attitude_estimation) const{
    //desired_acceleration means the desired thrust and is perpendicular to the body frame.
    const Eigen::Quaterniond q_heading = Eigen::Quaterniond(
      Eigen::AngleAxisd(referance_heading,Eigen::Vector3d::UnitZ()));

    const Eigen::Vector3d x_C = q_heading * Eigen::Vector3d::UnitX();
    const Eigen::Vector3d y_C = q_heading * Eigen::Vector3d::UnitY();
    Eigen::Vector3d z_B ;
    if(almostZero(desired_acceleration.norm()))
    {// free fall
      z_B = attitude_estimation * Eigen::Vector3d::UnitZ();
    }else{
      z_B = desired_acceleration.normalized();
    }

    const Eigen::Vector3d x_B_prototype = y_C.cross(z_B);
    const Eigen::Vector3d x_B = computeRobustBodyXAxis(x_B_prototype, x_C, y_C, attitude_estimation);
    const Eigen::Vector3d y_B = (z_B.cross(x_B)).normalized();
    //from computed desired body axis can obtain desired rotation matrix
    const Eigen::Matrix3d R_W_B((Eigen::Matrix3d() << x_B, y_B, z_B).finished());
    const Eigen::Quaterniond desired_attitude(R_W_B);

    return desired_attitude;
  }

Eigen::Vector3d Controller::computeRobustBodyXAxis(
  const Eigen::Vector3d& x_B_prototype, const Eigen::Vector3d& x_C,
  const Eigen::Vector3d& y_C, const Eigen::Quaterniond& attitude_estimate) const{
    Eigen::Vector3d x_B = x_B_prototype;

    if(almostZero(x_B.norm()))
    {// y_c cross z_b = 0, means any x_B 
      //is lies autonomous in x_c-z_c plane

      //project x_b_estimated in the x_c-z_c plane
      const Eigen::Vector3d x_B_estimated = 
          attitude_estimate * Eigen::Vector3d::UnitX();
      const Eigen::Vector3d x_B_projected = 
          x_B_estimated - (x_B_estimated.dot(y_C))*y_C;
      
      if(almostZero(x_B_projected.norm())){
        // x_B_projected align y_c and z_b align y_c, should not happend
        x_B = x_C;
      }else{
        x_B = x_B_projected.normalized();
      }

    }else
    {
      x_B = x_B.normalized();
    }

    return x_B;
  }

bool Controller::almostZero(const double value) const{
	return fabs(value) < 0.001; 
}

bool Controller::almostZeroThrust(const double thrust) const{
  return fabs(thrust) < 0.01;
}

/* 
  compute u.thrust and u.q, controller gains and other parameters are in param_ 
*/
quadrotor_msgs::Px4ctrlDebug
Controller::calculateControl(const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu, 
    Controller_Output_t &u)
{
      /* WRITE YOUR CODE HERE */
      //compute disired acceleration
      /*step1
      Compute reference inputs
      */
      std::string constraint_info("");
  	  Eigen::Vector3d drag_accelerations = Eigen::Vector3d::Zero();
      Controller_Output_t reference_inputs;
      if(param_.perform_aerodynamics_compensation)
      {
        // Compute reference inputs that compensate for aerodynamic drag
      }
      else{
        //in this case not considered aerodynamic acceleration
        drag_accelerations = Eigen::Vector3d::Zero();
        reference_inputs = computeNominalReferenceInputs(des,odom);

      }

      /*
	    step2 get the pid error
	    */

     Eigen::Vector3d e_p, e_v;
     double e_yaw = 0.0;
     // only  used in hovering control
    // if(des.v(0)!=0 || des.v(1)!=0 || des.v(2) != 0){
    //   int_e_v.setZero();
    // }

    double yaw_curr = get_yaw_from_quaternion(odom.q);
    double yaw_des = des.yaw;
    

       // Compute desired control commands
    Eigen::Vector3d pid_error_accelerations;
    if(param_.attitude_fb == 1)
      pid_error_accelerations = computePIDErrorAcc(des,odom,imu);
    else{
      pid_error_accelerations = computePIDErrorAccZJU(des,odom,imu);
    }
    Eigen::Vector3d Ka, Kp;
    Eigen::Vector3d des_acc;
    Ka << param_.gain.Ka0, param_.gain.Ka1, param_.gain.Ka2;
    
    // cal hov thr 
    Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
    Eigen::Vector3d u_p = Kp.asDiagonal() * (des.p - odom.p);
    u.des_v_real = des.v + u_p; // For estimating hover percent

    des_acc = Ka.asDiagonal() * des.a + pid_error_accelerations;
    des_acc += Eigen::Vector3d(0,0,param_.gra);

    //u.thrust = computeDesiredCollectiveThrustSignal(des_acc);
    //cal thr
    Eigen::Matrix3d wRb_odom = odom.q.toRotationMatrix();
    Eigen::Vector3d z_b_curr = wRb_odom.col(2);
    Eigen::Vector3d F_des = des_acc*param_.mass;
    double u1 = F_des.dot(z_b_curr);
    double fullparam = 0.7;
    u.thrust = u1 / param_.full_thrust;
    if(u.thrust>=fullparam)
      ROS_WARN("FULL THRUST");
    u.thrust = u.thrust>=fullparam?fullparam:u.thrust;
    //thr
    
    const Eigen::Quaterniond desired_attitude = computeDesiredAttitude(des_acc, des.yaw,odom.q);
	  const Eigen::Vector3d feedback_bodyrates = computeFeedBackControlBodyrates(desired_attitude,odom.q);

    if(param_.use_bodyrate_ctrl)
    {
      u.bodyrates = reference_inputs.bodyrates + feedback_bodyrates;
    }else{
      u.q = desired_attitude;

      // In ATTITUDE control mode the x-y-bodyrates contain just the feed forward
    // terms. The z-bodyrate has to be from feedback control
      u.bodyrates = reference_inputs.bodyrates;
      u.bodyrates.z() += feedback_bodyrates.z();
    }

    


  /* WRITE YOUR CODE HERE */

  //used for debug
  // debug_msg_.des_p_x = des.p(0);
  // debug_msg_.des_p_y = des.p(1);
  // debug_msg_.des_p_z = des.p(2);
  
  debug_msg_.des_v_x = des.v(0);
  debug_msg_.des_v_y = des.v(1);
  debug_msg_.des_v_z = des.v(2);
  
  debug_msg_.des_a_x = des_acc(0);
  debug_msg_.des_a_y = des_acc(1);
  debug_msg_.des_a_z = des_acc(2);
  
  debug_msg_.des_q_x = u.q.x();
  debug_msg_.des_q_y = u.q.y();
  debug_msg_.des_q_z = u.q.z();
  debug_msg_.des_q_w = u.q.w();
  
  debug_msg_.des_thr = u.thrust;
  
  // Used for thrust-accel mapping estimation
  timed_thrust_.push(std::pair<ros::Time, double>(ros::Time::now(), u.thrust));
  while (timed_thrust_.size() > 100)
  {
    timed_thrust_.pop();
  }
  return debug_msg_;
}

// /*
//   compute throttle percentage 
// */
// double 
// Controller::computeDesiredCollectiveThrustSignal(
//     const Eigen::Vector3d &des_acc)
// {
//   double throttle_percentage(0.0);
  
//   /* compute throttle, thr2acc has been estimated before */
//   throttle_percentage = des_acc(2) / thr2acc_;

//   return throttle_percentage;
// }

// bool 
// Controller::estimateThrustModel(
//     const Eigen::Vector3d &est_a,
//     const Parameter_t &param)
// {
//   ros::Time t_now = ros::Time::now();
//   while (timed_thrust_.size() >= 1)
//   {
//     // Choose data before 35~45ms ago
//     std::pair<ros::Time, double> t_t = timed_thrust_.front();
//     double time_passed = (t_now - t_t.first).toSec();
//     if (time_passed > 0.045) // 45ms
//     {
//       // printf("continue, time_passed=%f\n", time_passed);
//       timed_thrust_.pop();
//       continue;
//     }
//     if (time_passed < 0.035) // 35ms
//     {
//       // printf("skip, time_passed=%f\n", time_passed);
//       return false;
//     }

//     /***********************************************************/
//     /* Recursive least squares algorithm with vanishing memory */
//     /***********************************************************/
//     double thr = t_t.second;
//     timed_thrust_.pop();
    
//     /***********************************/
//     /* Model: est_a(2) = thr1acc_ * thr */
//     /***********************************/

//     double gamma = 1 / (rho2_ + thr * P_ * thr);
//     double K = gamma * P_ * thr;
//     thr2acc_ = thr2acc_ + K * (est_a(2) - thr * thr2acc_);
//     P_ = (1 - K * thr) * P_ / rho2_;
    
//     //fflush(stdout);

//     // debug_msg_.thr2acc = thr2acc_;
//     return true;
//   }
//   return false;
// }

// void 
// Controller::resetThrustMapping(void)
// {
//   thr2acc_ = param_.gra / param_.thr_map.hover_percentage;
//   P_ = 1e6;
// }

Eigen::Vector3d Controller::computePIDErrorAcc(
    const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu) const {
  // Compute the desired accelerations due to control errors in world frame
  // with a PID controller
  Eigen::Vector3d acc_err;
  Eigen::Vector3d Kp,Kv;
  Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
  Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
  acc_err = Kv.asDiagonal() * (des.v - odom.v) + Kp.asDiagonal() * (des.p - odom.p);

  return acc_err;
}


Eigen::Vector3d Controller::computePIDErrorAccZJU(
    const Desired_State_t &des,
    const Odom_Data_t &odom,
    const Imu_Data_t &imu) {
  // Compute the desired accelerations due to control errors in world frame
  // with a PID controller
  Eigen::Vector3d acc_err;
  Eigen::Vector3d Kp, Kv, Kvi;
  Kp << param_.gain.Kp0, param_.gain.Kp1, param_.gain.Kp2;
  Kv << param_.gain.Kv0, param_.gain.Kv1, param_.gain.Kv2;
  Kvi << param_.gain.Kvi0, param_.gain.Kvi1, param_.gain.Kvi2;
  
  Eigen::Vector3d e_p, e_v, F_des;
	double e_yaw = 0.0;
	if (des.v(0) != 0.0 || des.v(1) != 0.0 || des.v(2) != 0.0) {
		// ROS_INFO("Reset integration");
    int_e_v.setZero();
	}
  double yaw_curr = get_yaw_from_quaternion(odom.q);
	double	yaw_des = des.yaw;

  Eigen::Matrix3d wRc = rotz(yaw_curr);
  Eigen::Matrix3d cRw = wRc.transpose();

  e_p = des.p - odom.p;
  Eigen::Vector3d u_p = wRc * Kp.asDiagonal() * cRw * e_p;
  // u.des_v_real = des.v + u_p; // For estimating hover percent
	e_v = des.v + u_p - odom.v;

  const std::vector<double> integration_enable_limits = {0.1, 0.1, 0.1};
	for (size_t k = 0; k < 3; ++k) {
		if (std::fabs(e_v(k)) < 0.2) {
			int_e_v(k) += e_v(k) * 1.0 / 50.0;
		}
	}
	Eigen::Vector3d u_v_p = wRc * Kv.asDiagonal() * cRw * e_v;
	const std::vector<double> integration_output_limits = {0.4, 0.4, 0.4};
	Eigen::Vector3d u_v_i = wRc * Kvi.asDiagonal() * cRw * int_e_v;
	for (size_t k = 0; k < 3; ++k) {
		if (std::fabs(u_v_i(k)) > integration_output_limits[k]) {
			myuav_utils::limit_range(u_v_i(k), integration_output_limits[k]);
			ROS_INFO("Integration saturate for axis %zu, value=%.3f", k, u_v_i(k));
		}
	}
	Eigen::Vector3d u_v = u_v_p + u_v_i;
	
  return u_v;
}

Eigen::Vector3d Controller::computeFeedBackControlBodyrates(const Eigen::Quaterniond& desired_attitude,
    const Eigen::Quaterniond& attitude_estimate){
		  // Compute the error quaternion
  const Eigen::Quaterniond q_e = attitude_estimate.inverse() * desired_attitude;
  // Compute desired body rates from control error
  Eigen::Vector3d bodyrates;
  double KAngR = param_.gain.KAngR;
  double KAngP = param_.gain.KAngP;
  double kAngY = param_.gain.KAngY;

  if (q_e.w() >= 0) {
    bodyrates.x() = 2.0 * KAngR * q_e.x();
    bodyrates.y() = 2.0 * KAngP * q_e.y();
    bodyrates.z() = 2.0 * kAngY * q_e.z();
  } else {
    bodyrates.x() = -2.0 * KAngR * q_e.x();
    bodyrates.y() = -2.0 * KAngP * q_e.y();
    bodyrates.z() = -2.0 * kAngY * q_e.z();
  }

  return bodyrates;
}

