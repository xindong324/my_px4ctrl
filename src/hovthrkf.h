#ifndef __HOVTHRKF_H
#define __HOVTHRKF_H

#incldue<Eigen/Dense>
#include<ros/ros.h>
#include<input.h>

class HovThrKF
{
public:
    Parameter_t& param_;
    ros::Publisher hov_thr_pub_;

    HovThrKF(Parameter_t&);
    void init();
    void process(double u);
    void update(double a);
    double get_hov_thr();
    void set_hov_thr(double hov);
    void simple_update(Eigen::Quaterniond q, double u, Eigen::Vector3d acc);
    void simple_update(Eigen::Vector3d des_v, Eigen::Vector3d odom_v);
private:
    // x is [thr, mass normalized collective thrust]
    Eigen::VectorXd x;
    Eigen::MatrixXd P;
    Eigen::MatrixXd Q;
    Eigen::MatrixXd F;
    Eigen::MatrixXd H;
    Eigen::MatrixXd B;
    Eigen::MatrixXd R;
}

#endif
