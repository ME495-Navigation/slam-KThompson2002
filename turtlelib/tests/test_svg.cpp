#include <catch2/catch_test_macros.hpp>
#include <catch2/matchers/catch_matchers_floating_point.hpp>

#include <sstream>
#include <string>
#include <iomanip>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"

using Catch::Matchers::WithinAbs;

using turtlelib::Point2D;
using turtlelib::Vector2D;
using turtlelib::Svg;

static constexpr double EPS = 1e-6;

namespace
{
    std::ostringstream make_svg_stream()
    {
        std::ostringstream out;
        out << std::fixed << std::setprecision(6);
        return out;
    }
}

TEST_CASE("Svg core functions", "[svg]")
{
    // matches your default mapping assumptions in svg.cpp:
    // viewBox width  = 8.5*96 = 816
    // viewBox height = 11 *96 = 1056
    // center is (408, 528)
    Svg svg{8.5, 11.0, 96.0};

    SECTION("point_to_svg and vec_to_svg map correctly")
    {
        const Point2D origin_svg = svg.point_to_svg(Point2D{0.0, 0.0});
        REQUIRE_THAT(origin_svg.x, WithinAbs(408.0, EPS));
        REQUIRE_THAT(origin_svg.y, WithinAbs(528.0, EPS));

        const Point2D p_svg = svg.point_to_svg(Point2D{1.0, 2.0});
        REQUIRE_THAT(p_svg.x, WithinAbs(504.0, EPS));  // 408 + 96
        REQUIRE_THAT(p_svg.y, WithinAbs(336.0, EPS));  // 528 - 192

        const Vector2D v_svg = svg.vec_to_svg(Vector2D{1.0, 2.0});
        REQUIRE_THAT(v_svg.x, WithinAbs(96.0, EPS));
        REQUIRE_THAT(v_svg.y, WithinAbs(-192.0, EPS));
    }

    SECTION("draw_point outputs correctly formatted circle tag")
    {
        auto out = make_svg_stream();
        svg.draw_point(out, Point2D{1.0, -1.0}, "purple");

        // (1,-1) -> (408+96, 528 - (-96)) = (504, 624)
        const std::string expected =
            "<circle cx=\"504.000000\" cy=\"624.000000\" r=\"3\" stroke=\"purple\" "
            "fill=\"purple\" stroke-width=\"1\"/>\n";

        REQUIRE(out.str() == expected);
    }

    SECTION("draw_vector outputs correctly formatted line tag")
    {
        auto out = make_svg_stream();
        svg.draw_vector(out, Point2D{0.0, 0.0}, Vector2D{1.0, 0.0}, "black");

        // tail_svg = (408,528), head_svg = (504,528)
        const std::string expected =
            "<line x1=\"504.000000\" x2=\"408.000000\" y1=\"528.000000\" y2=\"528.000000\" "
            "stroke=\"black\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\"/>\n";

        REQUIRE(out.str() == expected);
    }
}
