cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(uav_plugin)

# Generate compile_commands.json for IDE/linter
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(gazebo REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(uav_plugin SHARED UavPlugin.cpp KrenCtrl.cpp KrenMdl.cpp)
target_link_libraries(uav_plugin ${GAZEBO_LIBRARIES})

add_library(random_robot_plugin SHARED RandomRobotPlugin.cpp)
target_link_libraries(random_robot_plugin ${GAZEBO_LIBRARIES})
