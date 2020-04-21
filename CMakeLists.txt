cmake_minimum_required(VERSION 2.8.3)
project(chicken_head)

find_package(catkin REQUIRED
  COMPONENTS
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
  DEPENDS
)

install(PROGRAMS
  scripts/chicken_head.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)