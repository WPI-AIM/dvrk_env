cmake_minimum_required(VERSION 2.8.3)
project(dvrk_model)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS dvrk ecm cart mtm psm Tool_LND
  CATKIN_DEPENDS
)

