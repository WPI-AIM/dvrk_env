cmake_minimum_required(VERSION 2.8.3)
project(dvrk_model)

find_package(catkin REQUIRED)

catkin_package(
  INCLUDE_DIRS dvrk ECM Cart MTM PSM Tool_LND
  CATKIN_DEPENDS
)

