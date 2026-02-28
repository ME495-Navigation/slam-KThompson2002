cmake_minimum_required(VERSION 3.20)
project(nuslam)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(turtlelib REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Armadillo REQUIRED)

add_executable(slam src/slam.cpp)
add_library(ekf src/ekf.cpp)
target_include_directories(ekf PUBLIC include)
target_link_libraries(ekf PUBLIC
  ${turtlelib_TARGETS}
  turtlelib::turtlelib
  Armadillo::Armadillo
)

target_link_libraries(slam PUBLIC
  ekf
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${turtlelib_TARGETS}
  Armadillo::Armadillo
  turtlelib::turtlelib
  rclcpp::rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

install(TARGETS
  slam
  DESTINATION lib/${PROJECT_NAME})

ament_package()
