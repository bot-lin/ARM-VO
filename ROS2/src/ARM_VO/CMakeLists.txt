cmake_minimum_required(VERSION 3.8)
project(ARM_VO)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS}  -O3 -std=c++11 -funroll-loops -Ofast")

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)

# OpenCV
find_package(OpenCV 3.3.1 QUIET)
if(NOT OpenCV_FOUND)
    find_package(OpenCV 4.0)
    if(NOT OpenCV_FOUND)
        message(FATAL_ERROR "OpenCV >= 3.3.1 not found")
    endif()
endif()
message(STATUS "Use OpenCV ${OpenCV_VERSION}")

if(NOT TARGET OpenMP::OpenMP_CXX)
    add_library(OpenMP_TARGET INTERFACE)
    add_library(OpenMP::OpenMP_CXX ALIAS OpenMP_TARGET)
    target_compile_options(OpenMP_TARGET INTERFACE ${OpenMP_CXX_FLAGS})
    find_package(Threads REQUIRED)
    target_link_libraries(OpenMP_TARGET INTERFACE Threads::Threads)
    target_link_libraries(OpenMP_TARGET INTERFACE ${OpenMP_CXX_FLAGS})
endif()

# uncomment the following section in order to fill in
# further dependencies manually.
# find_package(<dependency> REQUIRED)
set(armvo_Include /ARM-VO/include/)

include_directories(include 
                    ${OpenCV_INCLUDE_DIRS})
# file(GLOB SOURCES "src/vo_node.cpp" "../../../src/*.cpp")
file(GLOB SOURCES "src/vo_node.cpp")
# list(REMOVE_ITEM SOURCES ${CMAKE_CURRENT_SOURCE_DIR}/../../../src/main.cpp)

add_library(arm_vo_component SHARED
  ${SOURCES})
target_compile_definitions(arm_vo_component
  PRIVATE "ARM_VO_BUILDING_DLL")
ament_target_dependencies(arm_vo_component
  rclcpp 
  rclcpp_components
  sensor_msgs 
  image_transport
  cv_bridge
  OpenCV
  ) 
rclcpp_components_register_nodes(arm_vo_component "ARM_VO_Node")
set(node_plugins "${node_plugins}ARM_VO_Node;$<TARGET_FILE:arm_vo_component>\n")



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
  arm_vo_component
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

install(
  DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)
ament_export_libraries(arm_vo_component)


ament_package()
