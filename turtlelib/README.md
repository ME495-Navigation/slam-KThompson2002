# Turtlelib Setup
A library for performing 2D rigid body transofrmations and other geometry calculations.
* `cmake -B build` to build the cmake tools
* `cmake --build build` to recompile

Go into build and run 
* `ctest --verbose` to run the test functions

# Turtlelib Description
### 1. Angle Utilities (`angle.hpp`)
Provides mathematical utilities for handling angles and floating-point comparisons.
* **Angle Conversion**: Functions `rad2deg` and `deg2rad` for easy conversion between radians and degrees.
* **Normalization**: `normalize_angle` wraps any angle into the interval $(-\pi, \pi]$, ensuring consistent orientation logic.
* **Comparison**: `almost_equal` facilitates safe floating-point equality checks using an epsilon threshold.
* **Testing**: Includes `static_assert` verifications to ensure math correctness at compile time.

### 2. 2D Geometry Primitives (`geometry2d.hpp`)
Defines the fundamental building blocks for 2D space.
* **Structures**:
    * `Point2D`: Represents a location $(x, y)$ in space.
    * `Vector2D`: Represents a direction and magnitude $(x, y)$.
* **Operations**:
    * Vector arithmetic (subtraction of points, addition of vectors to points).
    * Vector normalization (`normalize`).
* **I/O**: Supports standard input/output streams and C++20 `std::format` for easy logging and parsing.

### 3. Special Euclidean Group SE(2) (`se2d.hpp`)
Handles rigid body transformations and kinematics in two dimensions.
* **Twist2D**: Represents velocity in 2D space (angular velocity $\omega$, linear velocities $v_x, v_y$).
* **Transform2D**: Represents a coordinate frame transformation (translation + rotation).
    * **Composition**: Multiply transforms (`*`, `*=`) to chain coordinate frame updates.
    * **Inversion**: Compute the inverse transform (`inv()`).
    * **Application**: Apply transformations to Points, Vectors, and Twists.
* **Formatting**: Robust `std::format` support for parsing and printing transformations with optional units (degrees vs radians).

### 4. SVG Visualization (`svg.hpp`)
A utility to visualize the geometry and math objects.
* **Svg Class**: Generates Scalable Vector Graphics (SVG) code to visualize the state of the robot.
* **Features**:
    * Draw individual `Point2D` and `Vector2D` objects.
    * Draw `Transform2D` coordinate frames (visualized as Red/Green axis arrows).
    * Automatically handles the conversion from internal "turtle units" to SVG pixel coordinates.

### 5. Differential Drive ('diff_drive.hpp)
A utility to complete inverse and forward dynamics of a differential drive robot
* **Wheel**: Represents the two wheel positions.
* **DiffDriuve**: Represents a Transform2D pose and the track width and wheel radius of the robot