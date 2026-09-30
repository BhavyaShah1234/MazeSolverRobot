# Minimum CMake version required to build this package.
cmake_minimum_required(VERSION 3.8)
# Declares this CMake project's name, matching the ROS2 package name.
project(maze_interfaces)

# ament's own CMake build system integration.
find_package(ament_cmake REQUIRED)
# Generates message/service/action code from .action/.msg/.srv files.
find_package(rosidl_default_generators REQUIRED)

# Generate the SolveMaze action's C++/Python bindings.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/SolveMaze.action"
)

# Ensure packages depending on this one also get the rosidl runtime.
ament_export_dependencies(rosidl_default_runtime)

# Only pull in linting when tests are enabled.
if(BUILD_TESTING)
  # ament's automatic lint-test discovery.
  find_package(ament_lint_auto REQUIRED)
  # Register whichever lint tests are enabled.
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize this as an ament package.
ament_package()
