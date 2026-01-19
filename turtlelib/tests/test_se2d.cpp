#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>
#include <catch2/catch_approx.hpp>

#include <sstream>
#include <string>
#include <numbers>
#include <format>

#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/angle.hpp"

using Catch::Matchers::WithinAbs;

static constexpr double EPS = 1e-6;

namespace
{
    void require_point_close(const turtlelib::Point2D & a, const turtlelib::Point2D & b, double eps = 1e-12)
    {
        REQUIRE_THAT(a.x, WithinAbs(b.x, eps));
        REQUIRE_THAT(a.y, WithinAbs(b.y, eps));
    }

    void require_vec_close(const turtlelib::Vector2D & a, const turtlelib::Vector2D & b, double eps = 1e-12)
    {
        REQUIRE_THAT(a.x, WithinAbs(b.x, eps));
        REQUIRE_THAT(a.y, WithinAbs(b.y, eps));
    }

    void require_twist_close(const turtlelib::Twist2D & a, const turtlelib::Twist2D & b, double eps = 1e-12)
    {
        REQUIRE_THAT(a.omega, WithinAbs(b.omega, eps));
        REQUIRE_THAT(a.x,     WithinAbs(b.x, eps));
        REQUIRE_THAT(a.y,     WithinAbs(b.y, eps));
    }
}

TEST_CASE("Transform2D constructors + accessors")
{
    using turtlelib::Transform2D;
    using turtlelib::Vector2D;

    SECTION("Default constructor is identity")
    {
        Transform2D I;
        require_vec_close(I.translation(), Vector2D{0.0, 0.0});
        REQUIRE_THAT(I.rotation(), WithinAbs(0.0, 1e-12));
    }

    SECTION("Translation-only constructor")
    {
        Transform2D T(Vector2D{1.5, -2.0});
        require_vec_close(T.translation(), Vector2D{1.5, -2.0});
        REQUIRE_THAT(T.rotation(), WithinAbs(0.0, 1e-12));
    }

    SECTION("Rotation-only constructor normalizes angle")
    {
        const double pi = std::numbers::pi;
        Transform2D R(3.0 * pi);
        REQUIRE_THAT(R.rotation(), WithinAbs(pi, 1e-12));
        require_vec_close(R.translation(), Vector2D{0.0, 0.0});
    }

    SECTION("Full constructor")
    {
        Transform2D tf(Vector2D{1.0, 2.0}, 0.25);
        require_vec_close(tf.translation(), Vector2D{1.0, 2.0});
        REQUIRE_THAT(tf.rotation(), WithinAbs(turtlelib::normalize_angle(0.25), 1e-12));
    }
}

TEST_CASE("Transform2D apply operator() for Point2D and Vector2D")
{
    using turtlelib::Transform2D;
    using turtlelib::Point2D;
    using turtlelib::Vector2D;

    const double theta = std::numbers::pi / 2.0;
    Transform2D tf(Vector2D{1.0, 2.0}, theta);

    SECTION("Apply to Vector2D rotates only")
    {
        Vector2D v{3.0, 4.0};
        Vector2D expected{-4.0, 3.0};
        require_vec_close(tf(v), expected);
    }

    SECTION("Apply to Point2D rotates and translates")
    {
        Point2D p{3.0, 4.0};
        Point2D expected{-3.0, 5.0};
        require_point_close(tf(p), expected);
    }
}

TEST_CASE("Transform2D inverse")
{
    using turtlelib::Transform2D;
    using turtlelib::Point2D;
    using turtlelib::Vector2D;

    Transform2D tf(Vector2D{1.0, -2.0}, 0.7);
    Transform2D inv = tf.inv();

    SECTION("inv undoes point transform")
    {
        Point2D p{0.25, -9.0};
        require_point_close(inv(tf(p)), p);
    }

    SECTION("inv undoes vector transform")
    {
        Vector2D v{5.0, 6.0};
        require_vec_close(inv(tf(v)), v);
    }
}

TEST_CASE("Transform2D composition: operator*= and operator*")
{
    using turtlelib::Transform2D;
    using turtlelib::Point2D;
    using turtlelib::Vector2D;

    Transform2D A(Vector2D{1.0, 2.0}, 0.3);
    Transform2D B(Vector2D{-4.0, 0.5}, -0.2);

    SECTION("operator*= matches applying sequentially")
    {
        Point2D p{1.0, 1.0};

        Transform2D C = A;
        C *= B;

        require_point_close(C(p), A(B(p)));
    }

    SECTION("operator* agrees with *=")
    {
        Transform2D C1 = A * B;
        Transform2D C2 = A;
        C2 *= B;

        REQUIRE_THAT(C1.rotation(), WithinAbs(C2.rotation(), 1e-12));
        require_vec_close(C1.translation(), C2.translation());
    }
}

TEST_CASE("Transform2D apply operator() for Twist2D (adjoint)")
{
    using turtlelib::Transform2D;
    using turtlelib::Twist2D;
    using turtlelib::Vector2D;

    const double theta = std::numbers::pi / 2.0;
    Transform2D tf(Vector2D{2.0, 3.0}, theta);

    SECTION("omega unchanged; v transforms as R v + p_perp * omega")
    {
        Twist2D tw{};
        tw.omega = 2.0;
        tw.x = 1.0;
        tw.y = -1.0;

        const turtlelib::Vector2D Rv{1.0, 1.0};
        const turtlelib::Vector2D pperp_w{-6.0, 4.0};

        Twist2D expected{};
        expected.omega = 2.0;
        expected.x = Rv.x + pperp_w.x; // -5
        expected.y = Rv.y + pperp_w.y; //  5

        require_twist_close(tf(tw), expected);
    }
}

TEST_CASE("Twist2D input with brackets", "[twist]") // Gregory, Aiosa
{
    std::stringstream ss;
    ss.str("<90 deg , 1.0, 2.0>"); // Test degrees input

    turtlelib::Twist2D tw;

    ss >> tw;

    REQUIRE(tw.omega == Catch::Approx(turtlelib::deg2rad(90.0)));
    REQUIRE(tw.x == Catch::Approx(1.0));
    REQUIRE(tw.y == Catch::Approx(2.0));

    ss.clear();
    ss.str("<1.57 rad, 3.0, -4.0>"); // Test radians input

    ss >> tw;
    REQUIRE(tw.omega == Catch::Approx(1.57));
    REQUIRE(tw.x == Catch::Approx(3.0));
    REQUIRE(tw.y == Catch::Approx(-4.0));

    ss.clear();
    ss.str("<0.5, -1.0, 2.0>"); // Test no unit input

    ss >> tw;
    REQUIRE(tw.omega == Catch::Approx(0.5));
    REQUIRE(tw.x == Catch::Approx(-1.0));
    REQUIRE(tw.y == Catch::Approx(2.0));
}

TEST_CASE("Twist2D input without brackets", "[twist]") // Gregory, Aiosa
{
    std::stringstream ss;
    ss.str("90 deg 1.0 2.0"); // Test degrees input

    turtlelib::Twist2D tw;

    ss >> tw;

    REQUIRE(tw.omega == Catch::Approx(turtlelib::deg2rad(90.0)));
    REQUIRE(tw.x == Catch::Approx(1.0));
    REQUIRE(tw.y == Catch::Approx(2.0));

    ss.clear();
    ss.str("1.57 rad 3.0 -4.0"); // Test radians input

    ss >> tw;
    REQUIRE(tw.omega == Catch::Approx(1.57));
    REQUIRE(tw.x == Catch::Approx(3.0));
    REQUIRE(tw.y == Catch::Approx(-4.0));

    ss.clear();
    ss.str("0.5 -1.0 2.0"); // Test no unit input

    ss >> tw;
    REQUIRE(tw.omega == Catch::Approx(0.5));
    REQUIRE(tw.x == Catch::Approx(-1.0));
    REQUIRE(tw.y == Catch::Approx(2.0));
}

TEST_CASE("Transform2D stream extraction operator>>")
{
    using turtlelib::Transform2D;
    using turtlelib::Vector2D;

    SECTION("Reads brace form {theta [unit], x, y}")
    {
        std::stringstream ss;
        ss << "{90 deg, 1.0, 2.0}";

        Transform2D tf;
        ss >> tf;

        REQUIRE(ss.good());
        REQUIRE_THAT(tf.rotation(), WithinAbs(std::numbers::pi / 2.0, 1e-12));
        require_vec_close(tf.translation(), Vector2D{1.0, 2.0});
    }

    SECTION("Reads whitespace form theta [unit] dx dy")
    {
        std::stringstream ss;
        ss << "3.141592653589793 rad 5 -6";

        Transform2D tf;
        ss >> tf;

        REQUIRE(ss.good());
        REQUIRE_THAT(tf.rotation(), WithinAbs(turtlelib::normalize_angle(std::numbers::pi), 1e-12));
        require_vec_close(tf.translation(), Vector2D{5.0, -6.0});
    }

    SECTION("Malformed brace input fails")
    {
        std::stringstream ss;
        ss << "{90 deg 1.0, 2.0}";

        Transform2D tf;
        ss >> tf;

        REQUIRE(ss.fail());
    }
}

TEST_CASE("std::format formatters for Transform2D and Twist2D")
{
    using turtlelib::Transform2D;
    using turtlelib::Twist2D;
    using turtlelib::Vector2D;

    SECTION("Transform2D formatter without unit (radians by default)")
    {
        Transform2D tf(Vector2D{1.0, 2.0}, 0.0);
        const std::string s = std::format("{:.1f}", tf);
        REQUIRE(s == "{0.0, 1.0, 2.0}");
    }

    SECTION("Transform2D formatter with degrees (D prefix)")
    {
        Transform2D tf(Vector2D{1.0, 2.0}, std::numbers::pi / 2.0);
        const std::string s = std::format("{:D.0f}", tf);
        REQUIRE(s == "{90 deg, 1, 2}");
    }

    SECTION("Twist2D formatter with rad/s (R prefix)")
    {
        Twist2D tw{1.0, 2.0, 3.0};
        const std::string s = std::format("{:R.1f}", tw);
        REQUIRE(s == "<1.0 rad/s, 2.0, 3.0>");
    }

    SECTION("Twist2D formatter with deg/s (D prefix)")
    {
        Twist2D tw{std::numbers::pi, 0.0, -1.0};
        const std::string s = std::format("{:D.0f}", tw);
        REQUIRE(s == "<180 deg/s, 0, -1>");
    }
}