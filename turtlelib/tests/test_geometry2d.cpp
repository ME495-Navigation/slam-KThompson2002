#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp> 
#include <sstream>   // std::stringstream
#include <cmath>

#include "turtlelib/geometry2d.hpp"


using turtlelib::Point2D;
using turtlelib::Vector2D;

TEST_CASE("Vector2D operations")
{
    SECTION("Point minus Point equals Vector2D")
    {
        Point2D a{3.0, 4.0};
        Point2D b{1.0, 1.5};

        Vector2D v = a - b;
        REQUIRE(v.x == Catch::Approx(2.0));
        REQUIRE(v.y == Catch::Approx(2.5));
    }

    SECTION("Point plus Vector equals Point")
    {
        Point2D p{1.0, 2.0};
        Vector2D d{3.0, -4.0};

        Point2D q = p + d;
        REQUIRE(q.x == Catch::Approx(4.0));
        REQUIRE(q.y == Catch::Approx(-2.0));
    }
}

TEST_CASE("normalize(Vector2D)")
{
    SECTION("Normalizes correctly")
    {
        Vector2D v{3.0, 4.0};
        Vector2D u = turtlelib::normalize(v);

        REQUIRE(u.x == Catch::Approx(0.6));
        REQUIRE(u.y == Catch::Approx(0.8));
    }

    SECTION("Zero vector throws")
    {
        Vector2D z{0.0, 0.0};
        REQUIRE_THROWS_AS(turtlelib::normalize(z), std::invalid_argument);
    }
}

TEST_CASE("Vector2D stream ouptut")
{
    Vector2D v{1.25, -3.5};
    std::ostringstream oss;
    oss << v;

    REQUIRE(oss.str() == "[1.25, -3.5]");
}

TEST_CASE("Vector2D stream input")
{
    SECTION("Reads bracket format")
    {
        std::stringstream ss;
        ss << "[1.25, -3.5]";

        Vector2D v{};
        ss >> v;

        REQUIRE(ss.good());
        REQUIRE(v.x == Catch::Approx(1.25));
        REQUIRE(v.y == Catch::Approx(-3.5));
    }

    SECTION("Reads plain format")
    {
        std::stringstream ss;
        ss << "1.25 -3.5";

        Vector2D v{};
        ss >> v;

        REQUIRE(ss);
        REQUIRE(v.x == Catch::Approx(1.25));
        REQUIRE(v.y == Catch::Approx(-3.5));
    }

    SECTION("Rejects malformed bracket input")
    {
        std::stringstream ss;
        ss << "[1.25 -3.5]";

        Vector2D v{};
        ss >> v;

        REQUIRE(ss.fail());
    }
}

TEST_CASE("Point2D stream input")
{
    SECTION("Reads paren format")
    {
        std::stringstream ss;
        ss << "(1.25, -3.5)";

        Point2D p{};
        ss >> p;

        REQUIRE(ss.good());
        REQUIRE(p.x == Catch::Approx(1.25));
        REQUIRE(p.y == Catch::Approx(-3.5));
    }

    SECTION("Reads plain format")
    {
        std::stringstream ss;
        ss << "1.25 -3.5";

        Point2D p{};
        ss >> p;
        
        REQUIRE(p.x == Catch::Approx(1.25));
        REQUIRE(p.y == Catch::Approx(-3.5));
    }

    SECTION("Rejects malformed paren input")
    {
        std::stringstream ss;
        ss << "(1.25 -3.5)"; // missing comma

        Point2D p{};
        ss >> p;

        REQUIRE(ss.fail());
    }
}

TEST_CASE("Vector2D arithmetic operators")
{
    SECTION("opeartor+= modifies lhs and returns reference")
    {
        Vector2D a{1.0, 2.0};
        Vector2D b{3.0, 4.5};

        Vector2D & ref = (a += b);

        REQUIRE(&ref == &a);
        REQUIRE(a.x == Catch::Approx(4.0));
        REQUIRE(a.y == Catch::Approx(6.5));
    }

    SECTION("operator+ returns sum and does not modify")
    {
        Vector2D a{1.0, 2.0};
        Vector2D b{3.0, 4.0};

        Vector2D c = a + b;

        REQUIRE(c.x == Catch::Approx(4.0));
        REQUIRE(c.y == Catch::Approx(6.0));

        REQUIRE(a.x == Catch::Approx(1.0));
        REQUIRE(a.y == Catch::Approx(2.0));
        REQUIRE(b.x == Catch::Approx(3.0));
        REQUIRE(b.y == Catch::Approx(4.0));
    }

    SECTION("operator-= modifies lhs and returns reference")
    {
        Vector2D a{5.0, -1.0};
        Vector2D b{2.0,  3.0};

        Vector2D & ref = (a -= b);

        REQUIRE(&ref == &a);
        REQUIRE(a.x == Catch::Approx(3.0));
        REQUIRE(a.y == Catch::Approx(-4.0));
    }

    SECTION("operator- returns difference and does not modify operands")
    {
        Vector2D a{5.0, -1.0};
        Vector2D b{2.0,  3.0};

        Vector2D c = a - b;

        REQUIRE(c.x == Catch::Approx(3.0));
        REQUIRE(c.y == Catch::Approx(-4.0));

        REQUIRE(a.x == Catch::Approx(5.0));
        REQUIRE(a.y == Catch::Approx(-1.0));
        REQUIRE(b.x == Catch::Approx(2.0));
        REQUIRE(b.y == Catch::Approx(3.0));
    }

    SECTION("operator*= scales vector in place and returns reference")
    {
        Vector2D v{1.5, -2.0};

        Vector2D & ref = (v *= 2.0);

        REQUIRE(&ref == &v);
        REQUIRE(v.x == Catch::Approx(3.0));
        REQUIRE(v.y == Catch::Approx(-4.0));
    }

    SECTION("operator* supports vector * scalar and scalar * vector")
    {
        Vector2D v{1.5, -2.0};

        Vector2D a = v * 2.0;
        REQUIRE(a.x == Catch::Approx(3.0));
        REQUIRE(a.y == Catch::Approx(-4.0));

        Vector2D b = 2.0 * v;
        REQUIRE(b.x == Catch::Approx(3.0));
        REQUIRE(b.y == Catch::Approx(-4.0));

        REQUIRE(v.x == Catch::Approx(1.5));
        REQUIRE(v.y == Catch::Approx(-2.0));
    }
}

TEST_CASE("Vector2D dot, magnitude, and angle")
{
    SECTION("dot product")
    {
        Vector2D a{3.0, 4.0};
        Vector2D b{1.0, 2.0};

        REQUIRE(turtlelib::dot(a, b) == Catch::Approx(11.0)); // 3*1 + 4*2

        Vector2D x{1.0, 0.0};
        Vector2D y{0.0, 1.0};
        REQUIRE(turtlelib::dot(x, y) == Catch::Approx(0.0)); // orthogonal
    }

    SECTION("magnitude")
    {
        Vector2D v{3.0, 4.0};
        REQUIRE(turtlelib::magnitude(v) == Catch::Approx(5.0));
    }

    SECTION("angle between vectors")
    {
        const double pi = std::acos(-1.0);

        Vector2D x{1.0, 0.0};
        Vector2D y{0.0, 1.0};
        REQUIRE(turtlelib::angle(x, y) == Catch::Approx(pi / 2.0));

        Vector2D a{2.0, 0.0};
        Vector2D b{5.0, 0.0};
        REQUIRE(turtlelib::angle(a, b) == Catch::Approx(0.0)); // same direction

        Vector2D c{-1.0, 0.0};
        REQUIRE(turtlelib::angle(x, c) == Catch::Approx(pi));  // opposite direction
    }
}