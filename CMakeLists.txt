cmake_minimum_required(VERSION 2.8.3)
project(roch_objects_detection)

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  roscpp
  sensor_msgs
  std_msgs
)

catkin_package(
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

## Declare a C++ executable
add_executable(roch_objects_detection_node src/simple_detection.cpp)


## Specify libraries to link a library or executable target against
target_link_libraries(roch_objects_detection_node
   ${catkin_LIBRARIES}
)



## Mark executables and/or libraries for installation
 install(TARGETS roch_objects_detection_node
   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
 )

## Mark other files for installation (e.g. launch and bag files, etc.)
 install(DIRECTORY launch
   DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
 )

