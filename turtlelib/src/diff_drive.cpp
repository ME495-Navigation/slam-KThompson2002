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
    }

    Wheel DiffDrive::inverseKinematics(const Twist2D & Vb) const
    {
    }
}