#ifndef TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP
#define TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP


#include<iosfwd>
/// NOTE: Include other needed headers here
#include <iosfwd>
#include <format>
#include <string>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/angle.hpp"
#include "turtlelib/se2d.hpp"



namespace turtlelib
{
    struct Wheel
    {
        /// \brief the right wheel position
        double right = 0.0;
        /// \brief the left wheel position
        double left = 0.0;
    };

    class DiffDrive
    {
    public:
        /// \brief Construct a diff drive robot model
        /// \param track distance between wheels
        /// \param radius wheel radius
        DiffDrive(double track, double radius, Transform2D pose = Transform2D(), Wheel wheels = Wheel{});

        /// \brief Get current pose 
        Transform2D pose() const;

        void setPose(const Transform2D & T);

        Wheel wheelAngles() const;

        void setWheelAngles(const Wheel & w);

        /// \brief get wheel geometry params 
        double track() const;
        double radius() const;

        /// \brief Reset both pose and wheel angles.
        void reset(const Transform2D & T_wb = Transform2D{}, const Wheel & w = Wheel{});

        /// \brief Forward kinematics:
        Twist2D forwardKinematics(const Wheel & new_wheels);

        /// \brief Inverse kinematics:
        Wheel inverseKinematics(const Twist2D & V) const;
    private:
        double wheel_track{0.0};
        double wheel_radius{0.0};
        
        Transform2D pose_;
        Wheel wheels;
    };
}

#endif // TURTLELIB_DIFF_DRIVE_INCLUDE_GUARD_HPP