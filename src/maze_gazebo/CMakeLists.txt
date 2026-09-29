cmake_minimum_required(VERSION 3.8)
project(maze_gazebo)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(Python3 REQUIRED COMPONENTS Interpreter)
find_package(realsense2_description REQUIRED)

# The overhead camera's visual is Intel's official D435 mesh. realsense2_description
# ships it as a DAE with no vertex normals, which Gazebo renders as a flat white
# silhouette, so convert the same triangles to an STL (which carries facet normals)
# at build time -- see scripts/convert_realsense_mesh.py.
get_filename_component(REALSENSE_SHARE "${realsense2_description_DIR}/.." ABSOLUTE)
set(D435_DAE ${REALSENSE_SHARE}/meshes/d435.dae)
set(D435_STL ${CMAKE_CURRENT_BINARY_DIR}/d435.stl)
add_custom_command(
  OUTPUT ${D435_STL}
  COMMAND ${Python3_EXECUTABLE} ${CMAKE_CURRENT_SOURCE_DIR}/scripts/convert_realsense_mesh.py
          ${D435_DAE} ${D435_STL}
  DEPENDS ${CMAKE_CURRENT_SOURCE_DIR}/scripts/convert_realsense_mesh.py ${D435_DAE}
  COMMENT "Converting the RealSense D435 mesh to STL (adding normals)"
)
add_custom_target(d435_mesh ALL DEPENDS ${D435_STL})

install(
  DIRECTORY
    config
    launch
    models
    worlds
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES ${D435_STL}
  DESTINATION share/${PROJECT_NAME}/models/overhead_camera/meshes
)

install(
  PROGRAMS scripts/referee_node.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
