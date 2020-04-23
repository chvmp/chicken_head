cmake_minimum_required(VERSION 2.8.3)
project(chicken_head)

find_package(catkin 
  REQUIRED
  COMPONENTS
  roscpp
  champ_msgs
  sensor_msgs
  geometry_msgs
)

catkin_package(
  LIBRARIES
  INCLUDE_DIRS
  CATKIN_DEPENDS
  roscpp
  champ_msgs
  sensor_msgs
  geometry_msgs
  DEPENDS
)

find_package(cmake_modules REQUIRED)
find_package(Eigen REQUIRED)
include_directories(${Eigen_INCLUDE_DIRS})

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  include_directories(${EIGEN_INCLUDE_DIRS})
)

add_library(chicken_head src/chicken_head.cpp)

add_executable(chicken_head_node src/chicken_head_node.cpp)
target_link_libraries(chicken_head_node chicken_head ${catkin_LIBRARIES})
add_dependencies(chicken_head_node ${catkin_EXPORTED_TARGETS})
add_dependencies(chicken_head ${catkin_EXPORTED_TARGETS})

install(TARGETS 
    chicken_head
    chicken_head_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)