#ifndef TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
#define TURTLELIB_GEOMETRY2D_HPP_INCLUDE_GUARD
/// \file
/// \brief Two-dimensional geometric primitives and other mathematical objects


// NOTE: Put additional include files here
#include<format>
#include <stdexcept>
// Note: <iosfwd> contains forward definitions for iostream objects
// allowing implementation of custom iostream operators without
// requiring the inclusion of <iostream>, which is a big header file
#include<iosfwd>
namespace turtlelib
{
    /// \brief a 2-Dimensional Point
    struct Point2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;
    };

    /// \brief Input a 2 dimensional point
    ///   You should be able to read vectors entered as follows:
    ///   "(x, y)" or "x y"  (Not including the "").
    /// \param is An istream from which to read
    /// \param p [out] The Point2D object that will store the input
    /// \returns A reference to is. An error flag is set on the stream if the input cannot be parsed.
    ///
    /// Hint: The following methods may be useful:
    /// https://en.cppreference.com/w/cpp/io/basic_istream/peek
    ///  .peek() looks at the next unprocessed character in the buffer without removing it
    /// https://en.cppreference.com/w/cpp/io/basic_istream/get
    ///  .get() removes the next unprocessed character from the buffer.
    ///
    /// You can also re-use operator>> from other types.
    ///
    /// The way input with istreams works is (more or less):
    /// What the user types is stored in a buffer until the user enters a newline (by pressing enter).
    /// The iostream methods then process the data in this buffer character-by-character.
    /// Typically, each character is examined and then removed from the buffer automatically.
    /// If the characters don't match what is expected (e.g., we expected an int but 'q' is encountered)
    /// an error flag is set on the stream object (e.g., std::cin).
    ///
    /// If you find yourself writing more than 30 or so lines for this function, you are likely
    /// on the wrong track.
    std::istream & operator>>(std::istream & is, Point2D & p);

    /// \brief A 2-Dimensional Vector
    struct Vector2D
    {
        /// \brief the x coordinate
        double x = 0.0;

        /// \brief the y coordinate
        double y = 0.0;
    };

    /// \brief Subtracting one point from another yields a vector
    /// \param head point corresponding to the head of the vector
    /// \param tail point corresponding to the tail of the vector
    /// \return a vector that points from p1 to p2
    /// NOTE: this operator is not implemented in terms of -=
    /// because subtracting two Point2D yields a Vector2D not a Point2D
    Vector2D operator-(const Point2D & head, const Point2D & tail);

    /// \brief Adding a vector to a point yields a new point displaced by the vector
    /// \param tail The origin of the vector's tail
    /// \param disp The displacement vector
    /// \return the point reached by displacing by disp from tail
    /// NOTE: this is not implemented in terms of += because of the different types
    Point2D operator+(const Point2D & tail, const Vector2D & disp);


    /// \brief output a 2 dimensional vector as [xcomponent, ycomponent]
    /// \param os - stream to output to
    /// \param v - the vector to print
    /// Note: Here to demonstrate the old method of custom output
    /// std::format is very recent so std::ostream is commonly used.
    /// DO NOT implement in terms of std::format, this is for you
    /// to have exposure to code that does not have std::format available.
    std::ostream & operator<<(std::ostream & os, const Vector2D & v);

    /// \brief input a 2 dimensional vector
    ///   You should be able to read vectors entered as follows:
    ///   "[x, y]" or "x y" (not including the "")
    /// \param is An istream from which to read
    /// \param v [out] - output vector
    /// \returns a reference to the istream, with any error flags set if
    /// a parsing error occurs
    std::istream & operator>>(std::istream & is, Vector2D & v);

    /// \brief Vector addition (in-place)
    Vector2D & operator+=(Vector2D & lhs, const Vector2D & rhs);

    /// \brief Vector addition
    Vector2D operator+(Vector2D lhs, const Vector2D & rhs);

    /// \brief Vector subtraction (in-place)
    Vector2D & operator-=(Vector2D & lhs, const Vector2D & rhs);

    /// \brief Vector subtraction
    Vector2D operator-(Vector2D lhs, const Vector2D & rhs);

    /// \brief Scalar multiplication (in-place)
    Vector2D & operator*=(Vector2D & v, double scalar);

    /// \brief Vector * scalar
    Vector2D operator*(Vector2D v, double scalar);

    /// \brief Scalar * vector
    Vector2D operator*(double scalar, Vector2D v);

    /// \brief Compute the dot product of two vectors
    /// \param v1 First vector
    /// \param v2 Second vector
    /// \returns v1 · v2
    double dot(const Vector2D & v1, const Vector2D & v2);

    /// \brief Compute the magnitude (length) of a vector
    /// \param v Vector
    /// \returns ||v||
    double magnitude(const Vector2D & v);

    /// \brief Return a unit vector in the direction of v
    /// \param in The vector to normalize
    /// \return The normalized vector.
    /// \throws std::invalid_input if in is the zero vector
    Vector2D normalize(Vector2D in);
}

/// \brief A Formatter for 2D points
/// The output is "(x, y)"
/// All floating-point format specifiers are honored and applied to both x and y.
template<class CharT>
class std::formatter<turtlelib::Point2D, CharT>
{
private:
    std::formatter<double, CharT> dbl_;

public:
    constexpr auto parse(std::basic_format_parse_context<CharT> & ctx)
    {
        return dbl_.parse(ctx);
    }

    template<class FormatContext>
    auto format(const turtlelib::Point2D & p, FormatContext & ctx) const
    {
        auto out = ctx.out();

        *out++ = CharT('(');
        ctx.advance_to(out);

        out = dbl_.format(p.x, ctx);
        ctx.advance_to(out);

        *out++ = CharT(',');
        *out++ = CharT(' ');
        ctx.advance_to(out);

        out = dbl_.format(p.y, ctx);
        ctx.advance_to(out);

        *out++ = CharT(')');
        return out;
    }
};

/// \brief A formatter for Vector2D
/// All double format-spec specifiers apply to each number in the vector
/// The vector is output as [x, y]
template<class CharT>
class std::formatter<turtlelib::Vector2D, CharT>
{
private:
    std::formatter<double, CharT> dbl_;

public:
    constexpr auto parse(std::basic_format_parse_context<CharT> & ctx)
    {
        return dbl_.parse(ctx);
    }

    template<class FormatContext>
    auto format(const turtlelib::Vector2D & v, FormatContext & ctx) const
    {
        auto out = ctx.out();

        *out++ = CharT('[');
        ctx.advance_to(out);

        out = dbl_.format(v.x, ctx);
        ctx.advance_to(out);

        *out++ = CharT(',');
        *out++ = CharT(' ');
        ctx.advance_to(out);

        out = dbl_.format(v.y, ctx);
        ctx.advance_to(out);

        *out++ = CharT(']');
        return out;
    }
};
#endif
