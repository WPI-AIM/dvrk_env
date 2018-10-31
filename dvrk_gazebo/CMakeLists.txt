cmake_minimum_required(VERSION 2.8.3)
project(dvrk_gazebo)

# Load catkin and all dependencies required for this package
find_package(catkin REQUIRED COMPONENTS
  roscpp
)

set (LIB_NAME dvrk_plugin)

# Depend on system install of Gazebo
find_package(gazebo REQUIRED)

#catkin_package(
#  INCLUDE_DIRS include
#)
link_directories(${GAZEBO_LIBRARY_DIRS})
include_directories(include)
include_directories(${Boost_INCLUDE_DIR} ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS})

add_library(${LIB_NAME} src/control_plugin.cpp)
target_link_libraries(${LIB_NAME} ${catkin_LIBRARIES} ${GAZEBO_LIBRARIES})

catkin_package(
  DEPENDS
    roscpp
)

set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
