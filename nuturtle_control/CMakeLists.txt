cmake_minimum_required(VERSION 3.20)
project(nuturtle_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nuturtlebot_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(std_srvs REQUIRED)
find_package(nuturtle_control_interfaces REQUIRED)
# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)

add_executable(turtle_control src/turtle_control.cpp)
add_executable(odometry src/odometry.cpp)
add_executable(circle src/circle.cpp)

target_link_libraries(turtle_control PUBLIC
  ${geometry_msgs_TARGETS}
  ${nuturtlebot_msgs_TARGETS}
  ${sensor_msgs_TARGETS}
  ${turtlelib_TARGETS}
  turtlelib::turtlelib
  rclcpp::rclcpp
)
target_link_libraries(odometry PUBLIC
  ${geometry_msgs_TARGETS}
  ${nuturtle_control_interfaces_TARGETS}
  ${turtlelib_TARGETS}
  ${sensor_msgs_TARGETS}
  ${nav_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
  turtlelib::turtlelib
  rclcpp::rclcpp
)
target_link_libraries(circle PUBLIC
  ${geometry_msgs_TARGETS}
  ${std_srvs_TARGETS}
  ${nuturtle_control_interfaces_TARGETS}
  rclcpp::rclcpp
)



if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  add_executable(turtle_control_test src/turtle_control_test.cpp)
  add_executable(turtle_odom_test_node src/turtle_odom_test_node.cpp)
  add_executable(turtle_circle_test src/turtle_circle_test.cpp)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
  find_package(Catch2 3 REQUIRED)
  find_package(catch_ros2 REQUIRED)
  target_link_libraries(turtle_control_test PUBLIC
    ${geometry_msgs_TARGETS}
    ${nuturtlebot_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${turtlelib_TARGETS}
    catch_ros2::catch_ros2_with_node_main
    turtlelib::turtlelib
    rclcpp::rclcpp
  )
  target_link_libraries(turtle_odom_test_node PUBLIC
    ${geometry_msgs_TARGETS}
    ${nuturtle_control_interfaces_TARGETS}
    ${turtlelib_TARGETS}
    ${sensor_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${tf2_TARGETS}
    ${tf2_ros_TARGETS}
    catch_ros2::catch_ros2_with_node_main
    turtlelib::turtlelib
    rclcpp::rclcpp
  )
  target_link_libraries(turtle_circle_test PUBLIC
    ${geometry_msgs_TARGETS}
    ${nuturtle_control_interfaces_TARGETS}
    catch_ros2::catch_ros2_with_node_main
    rclcpp::rclcpp
  )
  install(TARGETS
    turtle_control_test
    turtle_odom_test_node
    turtle_circle_test
    DESTINATION lib/${PROJECT_NAME}
  )
  catch_ros2_add_integration_test(turtle_control_test
    LAUNCH_FILE turtle_control_test.launch.xml
  )
  catch_ros2_add_integration_test(turtle_odom_test_node
    LAUNCH_FILE turtle_odom_test.launch.xml
  )
  catch_ros2_add_integration_test(turtle_circle_test
    LAUNCH_FILE turtle_circle_test.launch.xml
  )
endif()

install(TARGETS
  turtle_control
  odometry
  circle
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  config
  test
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
