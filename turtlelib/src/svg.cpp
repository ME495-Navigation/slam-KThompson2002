#include <sstream>
#include <string>
#include <vector>
#include <fstream>
#include <format>
#include <iostream>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"
#include "turtlelib/svg.hpp"

namespace turtlelib
{
    Svg::Svg(double width_in, double height_in, double px_per_unit)
    : vb_w_(width_in * px_per_unit),
      vb_h_(height_in * px_per_unit),
      w_in_(width_in),
      h_in_(height_in),
      px_(px_per_unit)
    {
    }
    Point2D Svg::point_to_svg(const Point2D& p) const
    {
        const double cx = vb_w_ / 2.0;
        const double cy = vb_h_ / 2.0;

        const double x_svg = cx + p.x * px_;
        const double y_svg = cy - p.y * px_; 

        return Point2D{x_svg, y_svg};
    }

    Vector2D Svg::vec_to_svg(const Vector2D& v) const
    {
        const double x_svg = v.x * px_;
        const double y_svg = -v.y * px_;       // flip y

        return Vector2D{x_svg, y_svg};
    }
    
    void Svg::write_header(std::ostream& out)
    {
        // Citation [Derek Dietz]
        // Header line
        out << "<svg width=\"8.500000in\" height=\"11.000000in\" "
            << "viewBox=\"0 0 816.000000 1056.000000\" "
            << "xmlns=\"http://www.w3.org/2000/svg\">\n";

        out << "<defs>\n";
        out << "  <marker\n";
        out << "     style=\"overflow:visible\"\n";
        out << "     id=\"Arrow1Sstart\"\n";
        out << "     refX=\"0.0\"\n";
        out << "     refY=\"0.0\"\n";
        out << "     orient=\"auto\">\n";
        out << "       <path\n";
        out << "         transform=\"scale(0.2) translate(6,0)\"\n";
        out << "         style=\"fill-rule:evenodd;fill:context-stroke;stroke:context-stroke;stroke-width:1.0pt\"\n";
        out << "         d=\"M 0.0,0.0 L 5.0,-5.0 L -12.5,0.0 L 5.0,5.0 L 0.0,0.0 z \"\n";
        out << "         />\n";
        out << "    </marker>\n";
        out << "</defs>\n";
    }

    void Svg::write_footer(std::ostream& out)
    {
        out << "</svg>";
    }
    // End Citation

    void Svg::draw_point(std::ostream& out, const Point2D & p, const std::string & color)
    {
        Point2D p_svg = point_to_svg(p);

        out << "<circle cx=\"" << p_svg.x
        << "\" cy=\"" << p_svg.y
        << "\" r=\"3\" stroke=\"" << color
        << "\" fill=\"" << color
        << "\" stroke-width=\"1\"/>\n";
    }

    void Svg::draw_vector(std::ostream& out, const Point2D & tail, const Vector2D & v, const std::string & color)
    {
        Point2D tail_svg = point_to_svg(tail);
        Vector2D v_svg = vec_to_svg(v);
        Point2D head_svg{tail_svg.x + v_svg.x, tail_svg.y + v_svg.y};

        out << "<line x1=\"" << head_svg.x
        << "\" x2=\"" << tail_svg.x
        << "\" y1=\"" << head_svg.y
        << "\" y2=\"" << tail_svg.y
        << "\" stroke=\"" << color
        << "\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\"/>\n";
    }

    void Svg::coordinateFrame(std::ostream& out, Transform2D& tf, const std::string& name)
    {
        // Frame origin in world coordinates
        Vector2D trans = tf.translation();
        Point2D tail{trans.x, trans.y};

        // Axis length in world units
        const double vector_len = 2.0;

        Vector2D x_axis_world{vector_len, 0.0};
        Vector2D y_axis_world{0.0, vector_len};


        Vector2D x_axis_body = tf(x_axis_world);
        Vector2D y_axis_body = tf(y_axis_world);

        Point2D Xtip_world{tail.x + x_axis_body.x, tail.y + x_axis_body.y};
        Point2D Ytip_world{tail.x + y_axis_body.x, tail.y + y_axis_body.y};

        Point2D tail_svg = point_to_svg(tail);
        Point2D x_tip_svg = point_to_svg(Xtip_world);
        Point2D y_tip_svg = point_to_svg(Ytip_world);


        // Output group
        out << "<g>\n";

        // X axis (red). Match draw_vector convention: x1=head, x2=tail, marker-start
        out << "<line x1=\"" << x_tip_svg.x
            << "\" x2=\"" << tail_svg.x
            << "\" y1=\"" << x_tip_svg.y
            << "\" y2=\"" << tail_svg.y
            << "\" stroke=\"red\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\"/>\n";

        // Y axis (green)
        out << "<line x1=\"" << y_tip_svg.x
            << "\" x2=\"" << tail_svg.x
            << "\" y1=\"" << y_tip_svg.y
            << "\" y2=\"" << tail_svg.y
            << "\" stroke=\"green\" stroke-width=\"5\" marker-start=\"url(#Arrow1Sstart)\"/>\n";

        // Label (match example: {name})
        out << "<text x=\"" << tail_svg.x 
        << "\" y=\"" << (tail_svg.y + 0.25)
        << "\">{" << name << "}</text>\n"; 
        out << "</g>\n";
    }
}