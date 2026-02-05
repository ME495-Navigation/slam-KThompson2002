#ifndef TURTLELIB_SVG_INCLUDE_GUARD_HPP
#define TURTLELIB_SVG_INCLUDE_GUARD_HPP
/// \file
/// \brief Visualization of the geometry code

#include <string>
#include <vector>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/se2d.hpp"

namespace turtlelib
{
    /// \brief A tiny SVG builder that draws points, vectors, and coordinate frames
    class Svg
    {
    public:
        Svg(double width_in = 8.5, double height_in = 11.0, double px_per_unit = 96.0);

        /// \brief Convert location of point to location in SVG
        Point2D point_to_svg(const Point2D& p) const;

        /// \brief Convert location of vector to location in SVG
        Vector2D vec_to_svg(const Vector2D& v) const;
        
        /// \brief Write header text for SVG
        // exposing this as the api requires the user to know that they should
        // write a header, write a footer, then call the drawing functions
        // so it's not a great abstraction and the class is not
        // maintaining the invariants of an svg
        void write_header(std::ostream& out);

        /// \brief Write footer text for SVG
        void write_footer(std::ostream& out);

        /// \brief Draw a point at a location
        void draw_point(std::ostream& out, const Point2D & p, const std::string & color = "purple");

        /// \brief Draw a vector starting at tail with direction/length v (turtle coords)
        void draw_vector(std::ostream& out, const Point2D & tail, const Vector2D & v, const std::string & color = "purple");

        /// \brief Draw a coordinate frame given by T (pose relative to fixed frame)
        /// Frame draws:
        ///  - red x-axis vector of length 1
        ///  - green y-axis vector of length 1
        ///  - text label "{name}" slightly offset from origin
        void coordinateFrame(std::ostream& out, Transform2D& tf, const std::string & name = "a");
    private:
        // viewBox dimensions in pixels
        double vb_w_{};
        double vb_h_{};

        // inches for width/height (only used for the <svg> width/height attributes)
        double w_in_{};
        double h_in_{};

        // pixels per turtle unit (1 unit == 1 inch in the example)
        double px_{};

        std::vector<std::string> elements_;

        // Convert turtle point/vector-space coordinate to viewBox pixel coordinate
        Point2D to_viewbox(const Point2D & p) const;

        // Midpoint of page in viewBox coords
        Point2D center_vb() const;
    };
}
#endif 
