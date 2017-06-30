cmake_minimum_required(VERSION 2.8.3)
project(cpp_sonar)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  sensor_msgs
  cv_bridge
  image_transport
)

# Determine if the BLUEVIEW_SDK_ROOT has been set
message("==================================")
if (DEFINED ENV{BLUEVIEW_SDK_ROOT})  
  message("Found Blueview SDK at:")
  message("$ENV{BLUEVIEW_SDK_ROOT}")  
else()
  message("WARNING: Can't find the BlueView SDK")
  message("Set the BLUEVIEW_SDK_ROOT environment variable in your .bashrc")
  message("if you require the syllo_blueview library")
  message("")
  return() # exit quietly if sdk not found
endif()

# Include the BLUEVIW SDK header files
include_directories($ENV{BLUEVIEW_SDK_ROOT}/include)

# Set the Blueview SDK libraries
set(BLUEVIEW_SDK_LIBS $ENV{BLUEVIEW_SDK_ROOT}/lib/libbvtsdk.so)


find_package(OpenCV)
include_directories(${OpenCV_INCLUDE_DIRS})

message("=======>")
message("${OpenCV_INCLUDE_DIRS}")

set( LIB_NAME "sdk_blueview" )


catkin_package(
  INCLUDE_DIRS include $ENV{BLUEVIEW_SDK_ROOT}/include
  LIBRARIES ${LIB_NAME}
  CATKIN_DEPENDS roscpp rospy std_msgs
)

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(include)
include_directories(
  ${catkin_INCLUDE_DIRS}
)

add_executable(main_cpp src/main.cpp)
add_executable(sub_cpp src/cpp_test_sub.cpp)

target_link_libraries(main_cpp
  ${BLUEVIEW_SDK_LIBS}
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  opencv_highgui
)

target_link_libraries(sub_cpp
  ${BLUEVIEW_SDK_LIBS}
  ${catkin_LIBRARIES} 
  ${OpenCV_LIBRARIES}
  opencv_highgui
)


add_dependencies(main_cpp test_publish_generate_messages_cpp)
add_dependencies(sub_cpp test_publish_generate_messages_cpp)
