#include "homemade_controller/forward_kinematics.hpp"

ForwardKinematics::ForwardKinematics(){
    angular_vel_vec_.reserve(4);
    angular_vel_vec_ = {0, 0, 0, 0};
  
}


ForwardKinematics::ForwardKinematics(double wheel_radius, double wheel_base){
    wheel_radius_ = wheel_radius;
    wheel_base_ = wheel_base;
    angular_vel_vec_.reserve(4);
    angular_vel_vec_ = {0, 0, 0, 0};
  

    
}

ForwardKinematics::~ForwardKinematics(){
    angular_vel_vec_.clear();
}

std::vector<double> ForwardKinematics::getWheelsAngularVelocities(double vx, double vy, double omega){

    double wl = omega * wheel_base_;

    angular_vel_vec_[0] = (wl + vy) / wheel_radius_;
    angular_vel_vec_[1] = (wl - vx) / wheel_radius_;
    angular_vel_vec_[2] = (wl - vy) / wheel_radius_;
    angular_vel_vec_[3] = (wl + vx) / wheel_radius_;
  

    return angular_vel_vec_;
}