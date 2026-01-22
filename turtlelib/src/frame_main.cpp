#include <iostream>
#include <fstream>
#include <string>
#include <cmath>

#include "turtlelib/se2d.hpp"
#include "turtlelib/geometry2d.hpp"
#include "turtlelib/svg.hpp"

namespace
{
    // Normalize a vector safely (uses your turtlelib normalize if you have it;
    // otherwise implement a simple one here).
    turtlelib::Vector2D unit_vector(const turtlelib::Vector2D & v)
    {
        const double n = std::hypot(v.x, v.y);
        if (n == 0.0) return turtlelib::Vector2D{0.0, 0.0};
        return turtlelib::Vector2D{v.x / n, v.y / n};
    }

    // Helper: print a vector in a readable way to stdout without relying on formatters
    void print_vec(const std::string & label, const turtlelib::Vector2D & v)
    {
        std::cout << label << " [" << v.x << ", " << v.y << "]\n";
    }

    // Helper: print a point similarly
    void print_pt(const std::string & label, const turtlelib::Point2D & p)
    {
        std::cout << label << " (" << p.x << ", " << p.y << ")\n";
    }
}

int main()
{
    using turtlelib::Transform2D;
    using turtlelib::Point2D;
    using turtlelib::Vector2D;

    std::cerr << "Enter transform T_ab (frame {b} in {a}): ";
    Transform2D T_ab;
    if (!(std::cin >> T_ab))
    {
        std::cerr << "Invalid input for T_ab\n";
        return 1;
    }

    std::cerr << "Enter transform T_bc (frame {c} in {b}): ";
    Transform2D T_bc;
    if (!(std::cin >> T_bc))
    {
        std::cerr << "Invalid input for T_bc\n";
        return 1;
    }

    const Transform2D T_ba = T_ab.inv();
    const Transform2D T_cb = T_bc.inv();

    const Transform2D T_ac = T_ab * T_bc;
    const Transform2D T_ca = T_ac.inv();

    // Output derived transforms (to stdout)
    std::cout << "T_ab computed/entered\n";
    std::cout << "T_bc computed/entered\n";
    std::cout << "T_ba = inv(T_ab)\n";
    std::cout << "T_cb = inv(T_bc)\n";
    std::cout << "T_ac = T_ab * T_bc\n";
    std::cout << "T_ca = inv(T_ac)\n";

    std::ofstream svg_file("/tmp/frames.svg");
    if (!svg_file)
    {
        std::cerr << "Failed to open /tmp/frames.svg for writing\n";
        return 1;
    }

    turtlelib::Svg svg;
    
    svg.write_header(svg_file);
    

    Transform2D T_aa;
    Transform2D tf_a = T_aa;
    Transform2D tf_b = T_ab;
    Transform2D tf_c = T_ac;

    svg.coordinateFrame(svg_file, tf_a, "a");
    svg.coordinateFrame(svg_file, tf_b, "b");
    svg.coordinateFrame(svg_file, tf_c, "c");

    std::cerr << "Enter a point p in frame {a}:\n";
    Point2D p_a;
    if (!(std::cin >> p_a))
    {
        std::cerr << "Invalid input for point p\n";
        return 1;
    }

    const Point2D p_b = T_ba(p_a);  // a -> b
    const Point2D p_c = T_ca(p_a);  // a -> c

    // Output points (stdout)
    std::cout << "Point locations:\n";
    print_pt("p_a =", p_a);
    print_pt("p_b =", p_b);
    print_pt("p_c =", p_c);

    // Draw p (world location), color-coded as requested
    svg.draw_point(svg_file, p_a, "purple"); // p in {a}
    svg.draw_point(svg_file, p_a, "brown");  // p in {b} (same world point, different semantic)
    svg.draw_point(svg_file, p_a, "orange"); // p in {c}

    // ---- Read vector v in frame {b} ----
    std::cerr << "Enter a vector v in frame {b}:\n";
    Vector2D v_b;
    if (!(std::cin >> v_b))
    {
        std::cerr << "Invalid input for vector v\n";
        return 1;
    }

    // Normalize v to v_hat (in frame {b})
    const Vector2D vhat_b = unit_vector(v_b);

    // Express vectors in world/{a} for drawing (b -> a is T_ab)
    const Vector2D v_a = T_ab(v_b);
    const Vector2D vhat_a = T_ab(vhat_b);

    // Draw v (brown) with tail at p (in world)
    svg.draw_vector(svg_file, p_a, v_a, "brown");

    // Draw v_hat (black) with tail at p (in world)
    svg.draw_vector(svg_file, p_a, vhat_a, "black");

    // Express v in frame {c} coordinates (b -> c is T_cb)
    const Vector2D v_c = T_cb(v_b);

    // Output v in frame {a} and {c} (stdout)
    std::cout << "Vector expressed in frames:\n";
    print_vec("v_a =", v_a);
    print_vec("v_c =", v_c);

    // Draw v with tail at p in frame {a}, purple (already have world vector v_a)
    svg.draw_vector(svg_file, p_a, v_a, "purple");

    // Draw v with tail at p in frame {c}, orange:
    // v_c is in {c}; convert to world by (c -> a) which is T_ac
    const Vector2D v_world_from_c = T_ac(v_c);
    svg.draw_vector(svg_file, p_a, v_world_from_c, "orange");

    // ---- Finish SVG ----
    svg.write_footer(svg_file);
    svg_file.close();

    std::cout << "Wrote SVG to /tmp/frames.svg\n";
    return 0;
}