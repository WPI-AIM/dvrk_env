cmake_minimum_required(VERSION 2.8.3)
project(dvrk_description)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS dvrk ecm suj mtm psm tool_lnd
  CATKIN_DEPENDS
)

