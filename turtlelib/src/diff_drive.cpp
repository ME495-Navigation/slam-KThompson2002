#include "turtlelib/diff_drive.hpp"

#include <cmath>
#include <stdexcept>

namespace turtlelib
{
    DiffDrive::DiffDrive(double track, double radius, Transform2D pose_in, Wheel wheels_in)
    : wheel_track(track),
    wheel_radius(radius),
    pose_(pose_in),
    wheels(wheels_in)
    {
        if (wheel_track <= 0.0) {
            throw std::logic_error("DiffDrive: track must be > 0");
        }
        if (wheel_radius <= 0.0) {
            throw std::logic_error("DiffDrive: radius must be > 0");
        }
    }
    // Getters and Setters
    Transform2D DiffDrive::pose() const
    {
        return pose_;
    }

    void DiffDrive::setPose(const Transform2D & T_wb)
    {
        pose_ = T_wb;
    }

    Wheel DiffDrive::wheelAngles() const
    {
        return wheels;
    }

    void DiffDrive::setWheelAngles(const Wheel & w)
    {
        wheels = w;
    }

    double DiffDrive::track() const
    {
        return wheel_track;
    }

    double DiffDrive::radius() const
    {
        return wheel_radius;
    }

    void DiffDrive::reset(const Transform2D & T_wb, const Wheel & w)
    {
        pose_ = T_wb;
        wheels = w;
    }

    Twist2D DiffDrive::forwardKinematics(const Wheel & new_wheels)
    {
        const double dphi_r = new_wheels.right - wheels.right;
        const double dphi_l = new_wheels.left  - wheels.left;

        Twist2D twist;
        twist.omega = wheel_radius * (dphi_r - dphi_l) / wheel_track;
        twist.x = wheel_radius * (dphi_r + dphi_l) / 2.0;
        twist.y = 0.0;

        const Transform2D twist_prime = integrate_twist(twist);
        pose_ = pose_ * twist_prime;
        wheels = new_wheels;
        return twist;
    }

    Wheel DiffDrive::inverseKinematics(const Twist2D & Vb) const
    {
        if (Vb.y > 0.0)
        {
            throw std::logic_error("Cannot achieve nonzero body y velocity without slip");
        }
        
        Wheel wdot;

        wdot.right = (Vb.x + Vb.omega * (wheel_track / 2.0)) / wheel_radius;
        wdot.left = (Vb.x - Vb.omega * (wheel_track / 2.0)) / wheel_radius;
        return wdot;
    }
}