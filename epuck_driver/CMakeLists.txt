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

## Declare a cpp executable
add_executable(epuck_driver 
	src/main.cpp 
	src/BasicCppDriver.cpp 
	src/BluetoothConnection.cpp 
	src/RosWrapper.cpp )

## Add cmake target dependencies of the executable/library
## as an example, message headers may need to be generated before nodes
# add_dependencies(epuck_driver_node epuck_driver_generate_messages_cpp)

## Specify libraries to link a library or executable target against
target_link_libraries(epuck_driver bluetooth ${catkin_LIBRARIES} ${OpenCV_LIBRARIES}
)

