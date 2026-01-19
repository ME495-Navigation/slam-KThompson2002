// src/converter.cpp
#include <iostream>
#include <iomanip>
#include <string>

#include "turtlelib/angle.hpp"
using std::cin;
using std::cout;
using std::string;


int main()
{
    
    while (true)
    {
        cout << "Enter an angle: <angle> <deg|rad>, (CTRL-D to exit)\n";

        double angle_in = 0.0;
        string unit;

        // Try to read <angle> <unit>
        if (!(cin >> angle_in >> unit))
        {
            // EOF: exit normally
            if (cin.eof())
            {
                return 0;
            }

            // malformed: clear error and discard rest of line
            cin.clear();
            string trash;
            std::getline(cin, trash);

            cout << "Invalid input: please enter <angle> <deg|rad>, (CTRL-D to exit)\n";
            continue;
        }

        // Validate unit
        if (unit != "deg" && unit != "rad")
        {
            // Discard rest of the line (if anything)
            string trash;
            std::getline(cin, trash);

            cout << "Invalid input: please enter <angle> <deg|rad>, (CTRL-D to exit)\n";
            continue;
        }

        // Print with decent numeric formatting
        cout << std::setprecision(15);

        if (unit == "deg")
        {
            // Normalize in degrees: (-180, 180]
            const double angle_deg_norm = turtlelib::rad2deg(turtlelib::normalize_angle(turtlelib::deg2rad(angle_in)));

            // Convert to radians and normalize: (-pi, pi]
            const double converted_rad_norm = turtlelib::normalize_angle(turtlelib::deg2rad(angle_deg_norm));

            cout << angle_deg_norm << " deg is " << converted_rad_norm << " rad.\n";
        }
        else // unit == "rad"
        {
            // Normalize in radians: (-pi, pi]
            const double angle_rad_norm = turtlelib::normalize_angle(angle_in);

            // Convert to degrees and normalize: (-180, 180]
            const double converted_deg_norm = turtlelib::rad2deg(turtlelib::normalize_angle(angle_rad_norm));

            cout << angle_rad_norm << " rad is " << converted_deg_norm << " deg.\n";
        }
    }
}
