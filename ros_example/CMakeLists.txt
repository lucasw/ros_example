cmake_minimum_required(VERSION 3.0.2)
project(ros_example)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  message_filters
  roscpp
)

catkin_package(
  CATKIN_DEPENDS
  roscpp
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(example
  src/example.cpp
)

target_link_libraries(example ${catkin_LIBRARIES})

install(TARGETS
  example
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
