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
find_package(sensor_msgs REQUIRED)
find_package(Armadillo REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(slam src/slam.cpp)
add_executable(landmark src/landmark.cpp)
add_library(ekf src/ekf.cpp)
add_library(circle src/circle.cpp)
target_include_directories(ekf PUBLIC include)
target_link_libraries(ekf PUBLIC
  ${turtlelib_TARGETS}
  ${ARMADILLO_LIBRARIES}
  turtlelib::turtlelib
)
target_include_directories(circle PUBLIC include)
target_link_libraries(circle PUBLIC
  ${turtlelib_TARGETS}
  ${ARMADILLO_LIBRARIES}
  turtlelib::turtlelib
)
# target_include_directories(circle PUBLIC include)
target_link_libraries(slam PUBLIC
  ekf
  ${nav_msgs_TARGETS}
  ${geometry_msgs_TARGETS}
  ${tf2_TARGETS}
  ${tf2_ros_TARGETS}
  ${turtlelib_TARGETS}
  ${sensor_msgs_TARGETS}
  ${ARMADILLO_LIBRARIES}
  ${visualization_msgs_TARGETS}
  turtlelib::turtlelib
  rclcpp::rclcpp
)
target_link_libraries(landmark PUBLIC
  circle
  ${sensor_msgs_TARGETS}
  ${visualization_msgs_TARGETS}
  ${turtlelib_TARGETS}
  turtlelib::turtlelib
  rclcpp::rclcpp
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(Catch2 REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()

  add_executable(test_ekf test/test_ekf.cpp)
  target_link_libraries(test_ekf ekf Catch2::Catch2WithMain)
  install(TARGETS test_ekf DESTINATION lib/${PROJECT_NAME})
  ament_add_test(test_ekf
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "$<TARGET_FILE:test_ekf>"
  )

  add_executable(circle_tests test/circle_tests.cpp)
  target_link_libraries(circle_tests circle Catch2::Catch2WithMain)
  install(TARGETS circle_tests DESTINATION lib/${PROJECT_NAME})
  ament_add_test(circle_tests
    GENERATE_RESULT_FOR_RETURN_CODE_ZERO
    COMMAND "$<TARGET_FILE:circle_tests>"
  )
endif()

install(TARGETS
  slam
  landmark
  DESTINATION lib/${PROJECT_NAME})

install(DIRECTORY launch config
  DESTINATION share/${PROJECT_NAME})

ament_package()
