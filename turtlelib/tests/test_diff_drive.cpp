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
        
    }
}

// TEST_CASE("Robot executes a pure rotation")
// {
//     SECTION("Forward Kinematics")
//     {

//     }
//     SECTION("Inverse Kinematics")
//     {
        
//     }
// }

// TEST_CASE("Robot follows the arc of a circle")
// {
//     SECTION("Forward Kinematics")
//     {

//     }
//     SECTION("Inverse Kinematics")
//     {
        
//     }
// }

// TEST_CASE("Impossible to follow twist")
// {
//     SECTION("Forward Kinematics")
//     {

//     }
//     SECTION("Inverse Kinematics")
//     {
        
//     }
// }