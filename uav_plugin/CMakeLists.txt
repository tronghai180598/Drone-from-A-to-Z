cmake_minimum_required(VERSION 3.0 FATAL_ERROR)
project(uav_plugin)

# Generate compile_commands.json for IDE/linter
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

find_package(gazebo REQUIRED)
find_package(SDL2 REQUIRED)

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(${SDL2_INCLUDE_DIRS})
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_library(uav_plugin SHARED UavPlugin.cpp KrenCtrl.cpp KrenMdl.cpp)
target_link_libraries(uav_plugin ${GAZEBO_LIBRARIES} ${SDL2_LIBRARIES})
