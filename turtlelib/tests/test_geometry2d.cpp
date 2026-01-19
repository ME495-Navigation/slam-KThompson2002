#include <catch2/catch_test_macros.hpp>
#include <catch2/catch_approx.hpp> 
#include <sstream>   // std::stringstream

#include "mylibrary/geometry2d.hpp"


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