cmake_minimum_required(VERSION 3.8)
project(ur_behavior)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(pyrobosim_ros REQUIRED)
find_package(rclpy REQUIRED)
find_package(ur_description REQUIRED)
find_package(ur_moveit_config REQUIRED)

install(PROGRAMS
  examples/demo_planning.py
  examples/load_pyrobosim_world.py
  examples/ur_interface.py
  examples/utils.py
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  data
  launch
  maps
  models
  worlds
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
