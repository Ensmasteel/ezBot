#ifndef FORWARD_KINEMATICS_HPP
#define FORWARD_KINEMATICS_HPP


#include <cmath>
#include <vector>
/*
forward kinematics of our robot
*/



class ForwardKinematics
{
    public:
        ForwardKinematics();
        ForwardKinematics(double wheel_radius, double wheel_base);
        ~ForwardKinematics();
        std::vector<double> getWheelsAngularVelocities(double vx, double vy, double omega);
    private:
        double wheel_radius_;
        double wheel_base_;
        std::vector<double> angular_vel_vec_;

};

#endif // FORWARD_KINEMATICS_HPP