#ifndef TURTLELIB_SE2_INCLUDE_GUARD_HPP
#define TURTLELIB_SE2_INCLUDE_GUARD_HPP
/// \file
/// \brief Two-dimensional rigid body transformations.


#include<iosfwd>
/// NOTE: Include other needed headers here
#include <iosfwd>
#include <format>
#include <string>

#include "turtlelib/geometry2d.hpp"
#include "turtlelib/angle.hpp"


namespace turtlelib
{

    /// \brief represent a 2-Dimensional twist
    struct Twist2D
    {
        /// \brief the angular velocity
        double omega = 0.0;

        /// \brief the linear x velocity
        double x = 0.0;

        /// \brief the linear y velocity
        double y = 0.0;
    };

    Twist2D & operator*=(Twist2D & lhs, double scalar);

    Twist2D operator*(Twist2D lhs, double scalar);

    /// \brief read the Twist2D in the format "<w [<unit>], x, y>" or as "w [<unit>] x y"
    /// The "" are not part of the input.
    /// The [<unit>] is optional and can be any string without spaces that starts with an r
    /// (for rad/s) and any string without spaces that starts with a d for deg/s)
    /// If the unit is omitted, assume rad/s.
    /// \param is [in/out] the istream to read from
    /// \param tw [out] the twist read from the stream
    /// \returns the istream is with the twist characters removed
    std::istream & operator>>(std::istream & is, Twist2D & tw);


    /// \brief a rigid body transformation in 2 dimensions
    class Transform2D
    {
    public:
        /// \brief Create an identity transformation
        Transform2D();

        /// \brief create a transformation that is a pure translation
        /// \param trans - the vector by which to translate
        explicit Transform2D(Vector2D trans);

        /// \brief create a pure rotation
        /// \param radians - angle of the rotation, in radians
        explicit Transform2D(double radians);

        /// \brief Create a transformation with a translational and rotational
        /// component
        /// \param trans - the translation
        /// \param radians - the rotation, in radians
        Transform2D(Vector2D trans, double radians);

        /// \brief apply a transformation to a 2D Point
        /// \param p the point to transform
        /// \return a point in the new coordinate system
        Point2D operator()(Point2D p) const;

        /// \brief apply a transformation to a 2D Vector
        /// \param v - the vector to transform
        /// \return a vector in the new coordinate system
        Vector2D operator()(Vector2D v) const;

        /// \brief apply a transformation to a Twist2D (e.g. using the adjoint)
        /// \param v - the twist to transform
        /// \return a twist in the new coordinate system
        Twist2D operator()(Twist2D v) const;

        /// \brief invert the transformation
        /// \return the inverse transformation.
        Transform2D inv() const;

        /// \brief compose this transform with another and store the result
        /// in this object
        /// \param rhs - the first transform to apply
        /// \return a reference to the newly transformed operator
        Transform2D & operator*=(const Transform2D & rhs);

        /// \brief the translational component of the transform
        /// \return the x,y translation
        Vector2D translation() const;

        /// \brief get the angular displacement of the transform
        /// \return the angular displacement, in radians
        double rotation() const;

        /// \brief see std::formatter for the Transform2D
        template<class CharT>
        friend struct std::formatter;

    private:
        Vector2D trans_{0.0, 0.0};
        double theta_{0.0}; // radians
    };


    /// \brief Read a transformation from stdin
    /// Should be able to read input either as:
    ///  "theta [<unit>] dx dy" (i.e., three numbers separated by whitespace, angle assumed to be radians)
    //   "{<angle> [<unit>], <x>, <y>}" (as output by std::format)
    ///  "{<angle> [<unit>], <x>, <y>}" (as output by std::format)
    ///  [<unit>] is optional and can be any string without spaces that starts with a d for deg or r for rad
    ///  If [<unit>] is omitted, assume the unit is radians
    std::istream & operator>>(std::istream & is, Transform2D & tf);

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs);

    /// \brief Compute the transform corresponding to integrating a constant body twist for 1 time-unit
    /// \param tw the body-frame twist (omega, x, y)
    /// \return Transform2D equal to exp(twist_hat * 1)
    Transform2D integrate_twist(Twist2D tw);
}

#define FORMAT_COMMA out = std::format_to(out, ", ");

/// \brief print the Twist2D as "<w [<unit>], x, y>"
/// An R at the beginning of the format-spec makes [<unit>] rad/s
/// A  D at the beginning of the format-spec makes [<unit>] deg/s
/// No R or D means no unit is printed but the w is taken to be in rad/s
///
/// After the optional "unit specifier" all double
/// format-spec values are accepted and apply to all numbers inserted
/// into the string.
template<class CharT>
class std::formatter<turtlelib::Twist2D, CharT>
{
private:
    enum class UnitMode { none, rad, deg };
    UnitMode mode_ = UnitMode::none;
    std::formatter<double, CharT> dbl_;

public:
    constexpr auto parse(std::basic_format_parse_context<CharT> & ctx)
    {
        auto it = ctx.begin();
        const auto end = ctx.end();

        if (it != end && (*it == CharT('R') || *it == CharT('r')))
        {
            mode_ = UnitMode::rad;
            ++it;
            ctx.advance_to(it);
        }
        else if (it != end && (*it == CharT('D') || *it == CharT('d')))
        {
            mode_ = UnitMode::deg;
            ++it;
            ctx.advance_to(it);
        }

        return dbl_.parse(ctx);
    }

    template<class FormatContext>
    auto format(const turtlelib::Twist2D & tw, FormatContext & ctx) const
    {
        auto out = ctx.out();
        out = std::format_to(out, "<");

        double w = tw.omega;
        if (mode_ == UnitMode::deg)
        {
            out = dbl_.format(turtlelib::rad2deg(w), ctx);
            out = std::format_to(out, " deg/s, ");
        }
        else if (mode_ == UnitMode::rad)
        {
            out = dbl_.format(w, ctx);
            out = std::format_to(out, " rad/s, ");
        }
        else {
            out = dbl_.format(w, ctx);
            FORMAT_COMMA
        }

        out = dbl_.format(tw.x, ctx);
        FORMAT_COMMA
        out = dbl_.format(tw.y, ctx);
        out = std::format_to(out, ">");
        return out;
    }
};

/// \brief A formatter for Transform2D
/// Creates a string representation of a Transform2D
/// as "{<angle> [<unit>], <x> <y>}"
/// An R at the beginning of the format-spec makes [<unit>] rad
/// A D  at the beginning of the format-spec makes [<unit>] deg
/// No R or D means no unit is printed but the angle is in radians.
///
/// After the optional "unit specifier" all double
/// format-spec values are accepted and apply to all numbers that
/// are put into the string
template<class CharT>
class std::formatter<turtlelib::Transform2D, CharT> : public std::formatter<turtlelib::Twist2D, CharT>
{
private:
    enum class UnitMode { none, rad, deg };
    UnitMode mode_ = UnitMode::none;
    std::formatter<double, CharT> dbl_;
public:
    constexpr auto parse(std::basic_format_parse_context<CharT> & ctx)
    {
        auto it = ctx.begin();
        const auto end = ctx.end();

        if (it != end && (*it == CharT('R') || *it == CharT('r')))
        {
            mode_ = UnitMode::rad;
            ++it;
            ctx.advance_to(it);
        }
        else if (it != end && (*it == CharT('D') || *it == CharT('d')))
        {
            mode_ = UnitMode::deg;
            ++it;
            ctx.advance_to(it);
        }

        // This is the key line: parse the numeric spec like ".1f"
        return dbl_.parse(ctx);
    }


    template<class FormatContext>
    auto format(const turtlelib::Transform2D & tf, FormatContext & ctx) const
    {
        auto out = ctx.out();

        out = std::format_to(out, "{{");

        if (mode_ == UnitMode::deg)
        {
            out = dbl_.format(turtlelib::rad2deg(tf.rotation()), ctx);
            out = std::format_to(out, " deg, ");
        }
        else if (mode_ == UnitMode::rad)
        {
            out = dbl_.format(tf.rotation(), ctx);
            out = std::format_to(out, " rad, ");
        }
        else
        {
            out = dbl_.format(tf.rotation(), ctx);
            FORMAT_COMMA
        }

        out = dbl_.format(tf.translation().x, ctx);
        FORMAT_COMMA
        out = dbl_.format(tf.translation().y, ctx);
        out = std::format_to(out, "}}");

        return out;
    }
};

#endif