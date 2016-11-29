cmake_minimum_required(VERSION 2.8.3)
project(epuck_driver)

## Find catkin macros and libraries
find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  tf
  cv_bridge
)
find_package(OpenCV REQUIRED)

################################################
## Declare ROS messages, services and actions ##
################################################

###################################
## catkin specific configuration ##
###################################
catkin_package(
   INCLUDE_DIRS include
   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp 
   DEPENDS opencv
)

# include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  include/epuck_driver
)


###########
## Build ##
###########

# Declare a C++ library
 add_library(epuck_driver
	src/BasicCppDriver.cpp 
	src/BluetoothConnection.cpp 
	src/RosWrapper.cpp
 )

## Specify libraries to link a library against
target_link_libraries(epuck_driver
  ${catkin_LIBRARIES}
  ${OpenCV_LIBRARIES}
  bluetooth
)


## Declare a cpp executable
add_executable(epuck_driver_node 
	src/main.cpp 
	 )

## Add cmake target dependencies of the executable
## same as for the library above
 add_dependencies(epuck_driver_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

## Specify libraries to link a library or executable target against
 target_link_libraries(epuck_driver_node
   epuck_driver
 )
