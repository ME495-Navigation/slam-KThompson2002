#include "turtlelib/geometry2d.hpp"

#include <istream>
#include <ostream>
#include <stdexcept>
#include <cmath>

namespace turtlelib
{
    std::istream & operator>>(std::istream & is, Point2D & p)
    {
        Point2D tmp = p;
        is >> std::ws;
        const int next = is.peek();
        if (next == '(')
        {
            char c = '\0';
            is.get(c);
            is >> std::ws;
            if (!(is >> tmp.x))
            {
                return is;
            }
            is >> std::ws;
            if (!(is.get(c)) || c != ',')  
            {
                is.setstate(std::ios::failbit);
                return is;
            }
            is >> std::ws;
            if (!(is >> tmp.y))  
            {
                return is;
            }
            is >> std::ws;
            if (!(is.get(c)) || c != ')') 
            {
                is.setstate(std::ios::failbit);
                return is;
            }
            p = tmp;
            return is;
        }
        else
        {
            if (!(is >> tmp.x >> tmp.y))
            {
                return is;
            }
            p = tmp;
            return is;
        }
    }

    Vector2D operator-(const Point2D & head, const Point2D & tail)
    {
        return Vector2D{head.x - tail.x, head.y - tail.y};
    }

    Point2D operator+(const Point2D & tail, const Vector2D & disp)
    {
        return Point2D{tail.x + disp.x, tail.y + disp.y};
    }

    std::ostream & operator<<(std::ostream & os, const Vector2D & v)
    {
        // Output exactly: [x, y]
        os << "[" << v.x << ", " << v.y << "]";
        return os;
    }

    std::istream & operator>>(std::istream & is, Vector2D & v)
    {
        Vector2D tmp = v;
        is >> std::ws;

        const int next = is.peek();
        if (next == '[')
        {
            char c = '\0';

            is.get(c);
            is >> std::ws;

            if (!(is >> tmp.x))
            {
                return is;
            }

            is >> std::ws;
            if (!(is.get(c)) || c != ',')
            {
                is.setstate(std::ios::failbit);
                return is;
            }

            is >> std::ws;
            if (!(is >> tmp.y))
            {
                return is;
            }

            is >> std::ws;
            if (!(is.get(c)) || c != ']')
            {
                is.setstate(std::ios::failbit);
                return is;
            }

            v = tmp;
            return is;
        }
        else
        {
            if (!(is >> tmp.x >> tmp.y))
            {
                return is;
            }
            v = tmp;
            return is;
        }
    }

    Vector2D operator+=(Vector2D & lhs, const Vector2D & rhs)
    {
        lhs.x += rhs.x;
        lhs.y += rhs.y;
        return lhs;
    }

    Vector2D operator+(Vector2D lhs, const Vector2D & rhs)
    {
        lhs += rhs;
        return lhs;
    }

    Vector2D & operator-=(Vector2D & lhs, const Vector2D & rhs)
    {
        lhs.x -= rhs.x;
        lhs.y -= rhs.y;
        return lhs;
    }

    Vector2D operator-(Vector2D lhs, const Vector2D & rhs)
    {
        lhs -= rhs;
        return lhs;
    }

    Vector2D & operator*=(Vector2D & v, double scalar)
    {
        v.x *= scalar;
        v.y *= scalar;
        return v;
    }

    Vector2D operator*(Vector2D v, double scalar)
    {
        v *= scalar;
        return v;
    }

    Vector2D operator*(double scalar, Vector2D v)
    {
        v *= scalar;
        return v;
    }

    double dot(const Vector2D & v1, const Vector2D & v2)
    {
        return v1.x * v2.x + v1.y * v2.y;
    }

    double magnitude(const Vector2D & v)
    {
        return std::sqrt(v.x * v.x + v.y * v.y);
    }

    Vector2D normalize(Vector2D in)
    {
        const double mag = std::sqrt(in.x * in.x + in.y * in.y);
        if (mag == 0.0)
        {
            throw std::invalid_argument("normalize(): zero vector");
        }
        in.x /= mag;
        in.y /= mag;
        return in;
    }
}