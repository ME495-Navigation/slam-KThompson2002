#ifndef TURTLELIB_ANGLE_HPP_INCLUDE_GUARD
#define TURTLELIB_ANGLE_HPP_INCLUDE_GUARD

/// \brief Functions for handling angles
/// NOTE: Include any needed header files here
#include <cmath>

namespace turtlelib
{
    /// \brief Approximately compare two floating-point numbers using
    ///        an absolute comparison
    /// \param d1 A number to compare
    /// \param d2 A second number to compare
    /// \param epsilon Absolute threshold required for equality
    /// \return true if abs(d1 - d2) < epsilon
    constexpr bool almost_equal(double d1, double d2, double epsilon=1.0e-12)
    {
        // HINT: This function must be implemented in the header file in order to be
        // used by other modules, because it is constexpr.
        // constexpr means that the function can be computed at compile time
        // if given compile-time constants as input, and therefore
        // it's definition must be visible in any compilation unit that uses it.
        // this extra ? : is unnecessary to return a bool of the condition
        return (d1 > d2) ? ((d1 - d2) < epsilon) : ((d2 - d1) < epsilon);
    }

    /// \brief Convert degrees to radians
    /// \param deg Angle in degrees
    /// \returns The equivalent angle in radians
    constexpr double deg2rad(double deg)
    {
        // HINT: C++20 #include<numbers> defines standard values
        // for many mathematical constants. Prior to C++20 you
        // would need to define your own constant for pi.
        // You should use the standardized value now
        using std::numbers::pi;
        return deg * (std::numbers::pi / 180.0);
    }

    /// \brief Convert radians to degrees
    /// \param rad  Angle in radians
    /// \returns The equivalent angle in degrees
    constexpr double rad2deg(double rad)
    {
        return rad * (180.0 / std::numbers::pi);
    }

    /// \brief Wrap an angle to (-PI, PI]
    /// \param rad Angle in radians
    /// \return An equivalent angle the range (-PI, PI]
    constexpr double normalize_angle(double rad)
    {
        // NOTE: You will receive partial credit only if this function uses loops.
        const double two_pi = 2.0 * std::numbers::pi; // auto
        rad = std::remainder(rad, two_pi); 
        if (almost_equal(rad, -std::numbers::pi, 1e-12)) {
            rad = std::numbers::pi;
        }
        return rad;
    }

    /// static_assertions test compile time assumptions.
    /// These tests can provide assurance that your code is correct at compile time!
    static_assert(almost_equal(0, 0), "is_zero failed");
    /// TASK: Write (at least) the following tests:
    /// 1. Compare two numbers where almost_equal is true or false depending on the epsilon argument
    ///    almost_equal(x, y, e1) == true
    ///    almost_equal(x, y, e2) == false
    static_assert( almost_equal(1.0, 1.0005, 1e-3) );   // diff = 0.0005 < 0.001  => true
    static_assert(!almost_equal(1.0, 1.0005, 1e-4) );
    /// 2. Compare negative and positive numbers that should not be equal
    static_assert(!almost_equal(-1.0, 1.0, 0.1) );    // diff = 2.0 >= 0.1 => false
    /// 3. Compare negative and positive numbers that should be equal
    static_assert( almost_equal(-0.0004, 0.0003, 1e-3) ); // diff = 0.0007 < 0.001 => true

    static_assert(almost_equal(deg2rad(0.0), 0.0), "deg2rad failed");
    /// TASK: Write at least 4 additional tests for deg2rad. Include at least one negative angle
    /// HINT: It helps to use angles where there is a simple known formula (e.g., 30 degrees)
    static_assert(almost_equal(deg2rad(180.0), std::numbers::pi), "deg2rad failed");
    static_assert(almost_equal(deg2rad(-90.0), -std::numbers::pi/2.0), "deg2rad failed");
    static_assert(almost_equal(deg2rad(30.0), std::numbers::pi/6.0), "deg2rad failed");
    static_assert(almost_equal(deg2rad(45.0), std::numbers::pi/4.0), "deg2rad failed");


    static_assert(almost_equal(rad2deg(0.0), 0.0), "rad2deg) failed");
    /// TASK: Write at least 4 additional tests for rad2deg. Include at least one negative angle
    /// HINT: It helps to use angles where there is a simple known formula (e.g., 30 degrees)
    static_assert(almost_equal(rad2deg(std::numbers::pi), 180.0), "rad2deg failed");
    static_assert(almost_equal(rad2deg(std::numbers::pi/2.0), 90.0), "rad2deg failed");
    static_assert(almost_equal(rad2deg(std::numbers::pi/6.0), 30.0), "rad2deg failed");
    static_assert(almost_equal(rad2deg(-std::numbers::pi/2.0), -90.0), "rad2deg failed");


    static_assert(almost_equal(normalize_angle(0.0), 0.0), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(std::numbers::pi), std::numbers::pi), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(-std::numbers::pi), std::numbers::pi), "norm_angle failed");
    /// Task: Write at least 8 additional tests for normalize_angle. This function is absolutely critical, so you want to get it right!
    /// Include at least once case  where the angle > 5.0*pi and one where the angle < -5.0*pi
    /// Also include: pi, -pi, -pi/4.0, 3*pi/2, and -5*pi/2.
    static_assert(almost_equal(normalize_angle(-std::numbers::pi/4.0), -std::numbers::pi/4.0), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(3.0*std::numbers::pi/2.0), -std::numbers::pi/2.0), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(-5.0*std::numbers::pi/2.0), -std::numbers::pi/2.0), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(4.0*std::numbers::pi), 0.0), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(-4.0*std::numbers::pi), 0.0), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(7.0*std::numbers::pi), std::numbers::pi), "norm_angle failed");        // > 5*pi
    static_assert(almost_equal(normalize_angle(-7.0*std::numbers::pi), std::numbers::pi), "norm_angle failed");      // < -5*pi
    static_assert(almost_equal(normalize_angle(5.0*(2.0*std::numbers::pi) + std::numbers::pi/3.0), std::numbers::pi/3.0), "norm_angle failed");
    static_assert(almost_equal(normalize_angle(-9.0*std::numbers::pi/2.0), -std::numbers::pi/2.0), "norm_angle failed");

}
#endif
