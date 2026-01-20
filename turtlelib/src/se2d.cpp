#include "turtlelib/se2d.hpp"

#include <cmath>
#include <istream>
#include <ostream>
#include <string>


namespace turtlelib                                
{
    namespace
    {
        /// \brief Read angle unit for the Twist2D and Transform2D istream functions
        /// \param is The input stream for Twist2D or Transform2D
        /// \param theta The input angle to be converted
        /// \return The final angle to be saved
        // Inline Citation [4] - Start
        static double read_angle_with_optional_unit(std::istream& is, double theta)
        {
            is >> std::ws;  // skip whitespace (but not punctuation)

            std::string unit;
            while (is.good()) {
                int c = is.peek();
                if (c == EOF) break;

                // Stop at punctuation (comma/brace/angle bracket/etc) because it's not alpha
                if (!std::isalpha(static_cast<unsigned char>(c))) break;

                unit.push_back(static_cast<char>(is.get()));  // consume the letter
            }

            // normalize to lowercase
            for (char& ch : unit) 
                ch = static_cast<char>(std::tolower(static_cast<unsigned char>(ch)));
            
            if (unit.empty() || unit[0] == 'r') return theta;
            if (unit[0] == 'd') return turtlelib::deg2rad(theta);

            is.setstate(std::ios::failbit);
            return theta;
        }
        // Inline Citation [4] - End

        
        bool expect(std::istream & is, char wanted)
        {
            char c = '\0';
            is >> std::ws;
            if (!is.get(c) || c != wanted)
            {
                is.setstate(std::ios::failbit);
                return false;
            }
            return true;
        }
    } // namespace

    /// \brief read the Twist2D in the format "<w [<unit>], x, y>" or as "w [<unit>] x y"
    /// The "" are not part of the input.
    /// The [<unit>] is optional and can be any string without spaces that starts with an r
    /// (for rad/s) and any string without spaces that starts with a d for deg/s)
    /// If the unit is omitted, assume rad/s.
    /// \param is [in/out] the istream to read from
    /// \param tw [out] the twist read from the stream
    /// \returns the istream is with the twist characters removed
    std::istream & operator>>(std::istream & is, Twist2D & tw)
    {
        Twist2D tmp = tw;
        is >> std::ws;

        if (is.peek() == '<')
        {
            if (!expect(is, '<')) return is;

            if (!(is >> tmp.omega)) return is;
            tmp.omega = read_angle_with_optional_unit(is, tmp.omega); // rad/s or deg/s -> rad/s

            if (!expect(is, ',')) return is;
            if (!(is >> tmp.x)) return is;

            if (!expect(is, ',')) return is;
            if (!(is >> tmp.y)) return is;

            if (!expect(is, '>')) return is;

            tw = tmp;
            return is;
        }

        // whitespace form: w [unit] x y
        if (!(is >> tmp.omega)) return is;
        tmp.omega = read_angle_with_optional_unit(is, tmp.omega);

        if (!(is >> tmp.x >> tmp.y)) return is;

        tw = tmp;
        return is;
    }

    /// \brief Create an identity transformation
    Transform2D::Transform2D() = default;

    /// \brief create a transformation that is a pure translation
    /// \param trans - the vector by which to translate
    Transform2D::Transform2D(Vector2D trans) : trans_(trans), theta_(0.0) {}

    /// \brief create a pure rotation
    /// \param radians - angle of the rotation, in radians
    Transform2D::Transform2D(double radians) : trans_{0.0, 0.0}, theta_(normalize_angle(radians)) {}

    /// \brief Create a transformation with a translational and rotational
    /// component
    /// \param trans - the translation
    /// \param radians - the rotation, in radians
    Transform2D::Transform2D(Vector2D trans, double radians)
    : trans_(trans), theta_(normalize_angle(radians))
    {
    }

    /// \brief apply a transformation to a 2D Point
    /// \param p the point to transform
    /// \return a point in the new coordinate system
    Point2D Transform2D::operator()(Point2D p) const
    {
        const double c = std::cos(theta_);
        const double s = std::sin(theta_);

        const double x = c * p.x - s * p.y + trans_.x;
        const double y = s * p.x + c * p.y + trans_.y;
        return Point2D{x, y};
    }

    /// \brief apply a transformation to a 2D Vector
    /// \param v - the vector to transform
    /// \return a vector in the new coordinate system
    Vector2D Transform2D::operator()(Vector2D v) const
    {
        const double c = std::cos(theta_);
        const double s = std::sin(theta_);

        const double x = c * v.x - s * v.y;
        const double y = s * v.x + c * v.y;
        return Vector2D{x, y};
    }

    /// \brief apply a transformation to a Twist2D (e.g. using the adjoint)
    /// \param v - the twist to transform
    /// \return a twist in the new coordinate system
    Twist2D Transform2D::operator()(Twist2D tw) const
    {
        // Ad_T [w; v] = [w; p_perp*w + R v]
        const Vector2D v{tw.x, tw.y};
        const Vector2D Rv = (*this)(v);

        // p_perp * w = [-p_y*w, p_x*w]
        const Vector2D p_perp_w{-trans_.y * tw.omega, trans_.x * tw.omega};

        return Twist2D{tw.omega, Rv.x + p_perp_w.x, Rv.y + p_perp_w.y};
    }

    /// \brief invert the transformation
    /// \return the inverse transformation.
    Transform2D Transform2D::inv() const
    {
        // Inverse: R^T, p' = -R^T p
        const double c = std::cos(theta_);
        const double s = std::sin(theta_);

        // R^T * p = [ c  s; -s  c ] * [px; py]
        const double x_rt =  c * trans_.x + s * trans_.y;
        const double y_rt = -s * trans_.x + c * trans_.y;

        return Transform2D(Vector2D{-x_rt, -y_rt}, -theta_);
    }

    /// \brief compose this transform with another and store the result
    /// in this object
    /// \param rhs - the first transform to apply
    /// \return a reference to the newly transformed operator
    Transform2D & Transform2D::operator*=(const Transform2D & rhs)
    {
        // Compose: T = this * rhs
        // R = R1 R2, p = p1 + R1 p2
        const Vector2D rotated_rhs_p = (*this)(rhs.trans_);
        trans_.x += rotated_rhs_p.x;
        trans_.y += rotated_rhs_p.y;

        theta_ = normalize_angle(theta_ + rhs.theta_);
        return *this;
    }

    /// \brief the translational component of the transform
    /// \return the x,y translation
    Vector2D Transform2D::translation() const { return trans_; }

    /// \brief get the angular displacement of the transform
    /// \return the angular displacement, in radians
    double Transform2D::rotation() const { return theta_; }

    /// \brief Read a transformation from stdin
    /// Should be able to read input either as:
    ///  "theta [<unit>] dx dy" (i.e., three numbers separated by whitespace, angle assumed to be radians)
    //   "{<angle> [<unit>], <x>, <y>}" (as output by std::format)
    ///  "{<angle> [<unit>], <x>, <y>}" (as output by std::format)
    ///  [<unit>] is optional and can be any string without spaces that starts with a d for deg or r for rad
    ///  If [<unit>] is omitted, assume the unit is radians
    std::istream & operator>>(std::istream & is, Transform2D & tf)
    {
        Transform2D tmp = tf;
        is >> std::ws;

        auto clear_eof_only = [&]() {
            if (is.eof() && !is.fail() && !is.bad()) {
                is.clear(is.rdstate() & ~std::ios::eofbit);
            }
        };

        if (is.peek() == '{')
        {
            if (!expect(is, '{')) return is;

            double theta = 0.0;
            double x = 0.0;
            double y = 0.0;

            if (!(is >> theta)) return is;
            theta = read_angle_with_optional_unit(is, theta);

            if (!expect(is, ',')) return is;
            if (!(is >> x)) return is;

            if (!expect(is, ',')) return is;
            if (!(is >> y)) return is;

            if (!expect(is, '}')) return is;

            tmp = Transform2D(Vector2D{x, y}, theta);
            tf = tmp;
            clear_eof_only();
            return is;
        }

        // whitespace form: theta [unit] dx dy
        double theta = 0.0;
        double dx = 0.0;
        double dy = 0.0;

        if (!(is >> theta)) return is;
        theta = read_angle_with_optional_unit(is, theta);

        if (!(is >> dx >> dy)) return is;

        tmp = Transform2D(Vector2D{dx, dy}, theta);
        tf = tmp;
        clear_eof_only();
        return is;
    }

    /// \brief multiply two transforms together, returning their composition
    /// \param lhs - the left hand operand
    /// \param rhs - the right hand operand
    /// \return the composition of the two transforms
    /// HINT: This function should be implemented in terms of *=
    Transform2D operator*(Transform2D lhs, const Transform2D & rhs)
    {
        lhs *= rhs;
        return lhs;
    }
}

