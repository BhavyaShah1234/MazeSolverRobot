# Minimum CMake version required to build this package.
cmake_minimum_required(VERSION 3.8)
# Declares this CMake project's name, matching the ROS2 package name.
project(maze_gazebo)

# Enable stricter warnings when building with GCC or Clang.
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # Turn on the standard extra-warnings flag set.
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ament's own CMake build system integration.
find_package(ament_cmake REQUIRED)
# Needed to invoke the mesh-conversion script at build time.
find_package(Python3 REQUIRED COMPONENTS Interpreter)
# Provides the RealSense D435's official DAE mesh, converted at build time below.
find_package(realsense2_description REQUIRED)

# The overhead camera's visual is Intel's official D435 mesh. realsense2_description
# ships it as a DAE with no vertex normals, which Gazebo renders as a flat white
# silhouette, so convert the same triangles to an STL (which carries facet normals)
# at build time -- see scripts/convert_realsense_mesh.py.
# Resolve realsense2_description's actual share directory from its CMake package dir.
get_filename_component(REALSENSE_SHARE "${realsense2_description_DIR}/.." ABSOLUTE)
# Path to the source DAE mesh.
set(D435_DAE ${REALSENSE_SHARE}/meshes/d435.dae)
# Path to the STL this build step produces.
set(D435_STL ${CMAKE_CURRENT_BINARY_DIR}/d435.stl)
# Custom build command: runs the Python converter script to produce the STL.
add_custom_command(
  # The file this command produces.
  OUTPUT ${D435_STL}
  # Run the mesh-conversion script with the DAE and STL paths.
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/convert_realsense_mesh.py
          ${D435_DAE} ${D435_STL}
  # Re-run this command if the script or the source mesh changes.
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/convert_realsense_mesh.py ${D435_DAE}
  # Human-readable message shown while this step runs.
  COMMENT "Converting the RealSense D435 mesh to STL (adding normals)"
)
# A build target that always runs, depending on the converted STL existing.
add_custom_target(d435_mesh ALL DEPENDS ${D435_STL})

# Install this package's non-Python resource directories.
install(
  DIRECTORY
    config
    launch
    models
    worlds
  DESTINATION share/${PROJECT_NAME}
)

# Install the build-time-generated STL mesh alongside the camera model.
install(
  FILES ${D435_STL}
  DESTINATION share/${PROJECT_NAME}/models/overhead_camera/meshes
)

# Install the referee node script as an executable.
install(
  PROGRAMS scripts/referee_node.py
  DESTINATION lib/${PROJECT_NAME}
)

# Only pull in linting when tests are enabled.
if(BUILD_TESTING)
  # ament's automatic lint-test discovery.
  find_package(ament_lint_auto REQUIRED)
  # Skip the copyright lint check for this package.
  set(ament_cmake_copyright_FOUND TRUE)
  # Skip the cpplint check for this package.
  set(ament_cmake_cpplint_FOUND TRUE)
  # Register whichever lint tests remain enabled.
  ament_lint_auto_find_test_dependencies()
endif()

# Finalize this as an ament package.
ament_package()
