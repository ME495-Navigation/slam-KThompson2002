#include "turtlelib/diff_drive.hpp"

#include <cmath>
#include <stdexcept>

namespace turtlelib
{
    // Getters and Setters
    Transform2D DiffDrive::pose() const
    {
        return T_wb_;
    }

    void DiffDrive::setPose(const Transform2D & T_wb)
    {
        T_wb_ = T_wb;
    }

    Wheel DiffDrive::wheelAngles() const
    {
        return wheels_;
    }

    void DiffDrive::setWheelAngles(const Wheel & w)
    {
        wheels_ = w;
    }

    double DiffDrive::track() const
    {
        return wheel_track_;
    }

    double DiffDrive::radius() const
    {
        return wheel_radius_;
    }


    Twist2D DiffDrive::forwardKinematics(const Wheel & new_wheels)
    {
        const double dphi_r = new_wheels.right - wheels.right;
        const double dphi_l = new_wheels.left  - wheels.left;

        Twist2D twist;
        twist.omega = wheel_radius * (dphi_r + dphi_l) / wheel_track;
        twist.x = wheel_radius * (dphi_r + dphi_l) / 2.0;
        twist.y = 0.0;

        const Transform2D twist_prime = integrate_twist(twist);
        pose = pose * twist_prime;
        wheels = new_wheels;
        return twist;

    Wheel DiffDrive::inverseKinematics(const Twist2D & Vb) const
    {
    }
}