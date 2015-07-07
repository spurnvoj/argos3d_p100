cmake_minimum_required(VERSION 2.8.3)
project(argos3d_p100)

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
#include_directories(include)

find_package( PCL REQUIRED )
include_directories(  ${catkin_INCLUDE_DIRS}  ${PCL_INCLUDE_DIRS} )
link_directories(${PCL_LIBRARY_DIRS})
link_directories(argos/lib/BtaP100/x64)
add_definitions(${PCL_DEFINITIONS})

## Declare a cpp executable
add_executable(argos3d_p100_node src/argos3d_p100_node.cpp src/visualizer.cpp src/time_measurement.cpp)

## Specify libraries to link a library or executable target against
target_link_libraries ( argos3d_p100_node ${PCL_LIBRARIES} ${catkin_LIBRARIES} bta_p100 SDL)
