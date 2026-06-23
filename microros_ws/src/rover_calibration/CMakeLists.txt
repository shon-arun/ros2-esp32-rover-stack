cmake_minimum_required(VERSION 3.8)
project(rover_calibration)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

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

# --- Square Driver Target ---
add_executable(square_driver src/square_driver.cpp)
ament_target_dependencies(square_driver rclcpp geometry_msgs nav_msgs)

# --- Straight Line Driver Target ---
add_executable(straight_line_driver src/straight_line_driver.cpp)
ament_target_dependencies(straight_line_driver rclcpp geometry_msgs nav_msgs)

# --- Rotation Driver Target ---
add_executable(rotation_driver src/rotation_driver.cpp)
ament_target_dependencies(rotation_driver rclcpp geometry_msgs nav_msgs tf2 tf2_geometry_msgs)

install(TARGETS
  square_driver
  straight_line_driver
  rotation_driver
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()