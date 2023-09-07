cmake_minimum_required(VERSION 3.0.2)
project(ros_example)

find_package(catkin REQUIRED COMPONENTS
  fkie_message_filters
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

add_executable(fkie_message_filters_sync
  src/fkie_message_filters_sync.cpp
)

target_link_libraries(fkie_message_filters_sync ${catkin_LIBRARIES})

add_executable(message_filters_sync
  src/message_filters_sync.cpp
)

target_link_libraries(message_filters_sync ${catkin_LIBRARIES})

add_executable(node
  src/node.cpp
)

target_link_libraries(node ${catkin_LIBRARIES})

install(TARGETS
  fkie_message_filters_sync
  message_filters_sync
  node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
