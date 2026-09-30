# Minimum CMake version required to build this package.
cmake_minimum_required(VERSION 3.8)
# Declares this CMake project's name, matching the ROS2 package name.
project(maze_bringup)

# ament's own CMake build system integration.
find_package(ament_cmake REQUIRED)

# Install this package's non-code resource directories.
install(
  DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

# Only pull in linting when tests are enabled.
if(BUILD_TESTING)
  # ament's automatic lint-test discovery.
  find_package(ament_lint_auto REQUIRED)
  # Register whichever lint tests are enabled.
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize this as an ament package.
ament_package()
