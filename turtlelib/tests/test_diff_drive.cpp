#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp> 

#include <cmath>
#include "turtlelib/diff_drive.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"

using turtlelib::DiffDrive;
using turtlelib::Transform2D;
using turtlelib::Twist2D;
using turtlelib::Wheel;

namespace
{
    constexpr double track = 0.16;
    constexpr double radius = 0.066;
    
    void pose_check(const Transform2D &T, double x, double y, double theta)
    {
        auto p = T.translation();
        REQUIRE(p.x == Catch::Approx(x));
        REQUIRE(p.y == Catch::Approx(y));
        REQUIRE(T.rotation() == Catch::Approx(theta));
    }
}

TEST_CASE("Robot drives forward")
{
    SECTION("Forward Kinematics")
    {
        DiffDrive diff(track, radius);
        diff.reset(Transform2D{}, Wheel{0.0, 0.0});

        Wheel new_w{1.0, 1.0};
        Twist2D Vb = diff.forwardKinematics(new_w);

        REQUIRE(Vb.omega == Catch::Approx(0.0));
        REQUIRE(Vb.y     == Catch::Approx(0.0));
        REQUIRE(Vb.x     == Catch::Approx(radius * 1.0));

        pose_check(diff.pose(), radius * 1.0, 0.0, 0.0);
    }
    SECTION("Inverse Kinematics")
    {
        DiffDrive diff(track, radius);
        diff.reset(Transform2D{}, Wheel{0.0, 0.0});

        Twist2D twist;
        twist.omega = 0.0;
        twist.x = 0.10; 
        twist.y = 0.0;

        Wheel wdot = diff.inverseKinematics(twist);

        REQUIRE(wdot.right == Catch::Approx(twist.x / radius));
        REQUIRE(wdot.left == Catch::Approx(twist.x / radius));

        Wheel new_w{wdot.right, wdot.left};
        diff.forwardKinematics(new_w);

        pose_check(diff.pose(), 0.10, 0.0, 0.0);
    }
}

TEST_CASE("Robot executes a pure rotation")
{
    SECTION("Forward Kinematics")
    {
        DiffDrive diff(track, radius);
        diff.reset(Transform2D{}, Wheel{0.0, 0.0});

        Wheel new_w{1.0, -1.0};
        Twist2D Vb = diff.forwardKinematics(new_w);

        REQUIRE(Vb.x == Catch::Approx(0.0));
        REQUIRE(Vb.y == Catch::Approx(0.0));
        REQUIRE(Vb.omega == Catch::Approx((2 * radius) / track));

        pose_check(diff.pose(), 0.0, 0.0, (2.0* radius) / track);
    }
    SECTION("Inverse Kinematics")
    {
        DiffDrive diff(track, radius);
        diff.reset(Transform2D{}, Wheel{0.0, 0.0});

        Twist2D twist;
        twist.omega = 1.0;
        twist.x = 0.0; 
        twist.y = 0.0;

        Wheel wdot = diff.inverseKinematics(twist);
        
        REQUIRE(wdot.right == Catch::Approx((twist.omega * track / 2.0) / radius));
        REQUIRE(wdot.left == Catch::Approx((-twist.omega * track / 2.0) / radius));
        diff.forwardKinematics(wdot);
        pose_check(diff.pose(), 0.0, 0.0, twist.omega);
    }
}

TEST_CASE("Robot follows the arc of a circle")
{
    SECTION("Forward Kinematics")
    {
        DiffDrive diff(track, radius);
        diff.reset(Transform2D{}, Wheel{0.0, 0.0});

        const double dphi_r = 1.5;
        const double dphi_l = 1.0;
        Wheel new_w;
        new_w.right = dphi_r;
        new_w.left  = dphi_l;


        Twist2D Vb = diff.forwardKinematics(new_w);

        const double dr = radius * dphi_r;
        const double dl = radius * dphi_l;

        const double w = (dr - dl) / track;
        const double v = (dr + dl) / 2.0;

        REQUIRE(Vb.omega == Catch::Approx(w));
        REQUIRE(Vb.x     == Catch::Approx(v));
        REQUIRE(Vb.y     == Catch::Approx(0.0));

        const double dx = (v / w) * std::sin(w);
        const double dy = (v / w) * (std::cos(w) - 1.0);

        pose_check(diff.pose(), dx, dy, w);
    }
    SECTION("Inverse Kinematics")
    {
        DiffDrive diff(track, radius);
        diff.reset(Transform2D{}, Wheel{0.0, 0.0});

        Twist2D twist;
        twist.omega = 0.5;
        twist.x = 0.2;
        twist.y = 0.0;
        
        Wheel wdot = diff.inverseKinematics(twist);

        Wheel new_w;
        new_w.right = wdot.right; 
        new_w.left  = wdot.left;
        diff.forwardKinematics(new_w);

        const double w = twist.omega;
        const double v = twist.x;
        const double dx = (v / w) * std::sin(w);
        const double dy = (v / w) * (std::cos(w) - 1.0);

        pose_check(diff.pose(), dx, dy, w);
    }
}

// TEST_CASE("Impossible to follow twist")
// {
//     SECTION("Inverse Kinematics")
//     {
        
//     }
// }