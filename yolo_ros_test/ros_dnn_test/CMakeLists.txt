cmake_minimum_required(VERSION 3.0.2)
project(ros_dnn_test)

## Compile as C++11, supported in ROS Kinetic and newer
#add_compile_options(-std=c++11)
set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED TRUE)

find_package(Boost REQUIRED COMPONENTS thread)

set(OpenCV_DIR /usr/local/lib/cmake/opencv4)
find_package(OpenCV REQUIRED)
message("OpenCV version: ${OpenCV_VERSION}")

include_directories(/usr/local/include/opencv4/opencv2)
include_directories(${OpenCV_INCLUDE_DIRS})

find_package(catkin REQUIRED COMPONENTS
  cv_bridge
  image_transport
  message_generation
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

add_message_files(
  FILES
  MsgState.msg
  BoundingBox.msg
  BoundingBoxes.msg
)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(
  INCLUDE_DIRS
    include
  LIBRARIES ros_dnn_test
  CATKIN_DEPENDS
    cv_bridge
    roscpp
    rospy
    std_msgs
    image_transport
  DEPENDS
    Boost
)

include_directories( ${Boost_INCLUDE_DIRS} ${catkin_INCLUDE_DIRS})

add_executable(yolo_node src/yolo_node.cpp)
add_dependencies(yolo_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(yolo_node 
    ${catkin_LIBRARIES}
    m
    pthread
    stdc++
    ${Boost_LIBRARIES}
    ${OpenCV_LIBRARIES}
    ${OpenCV_LIBS}
  )


add_executable(darknet_sub_node src/darknet_sub_node.cpp)
add_dependencies(darknet_sub_node ${catkin_EXPORTED_TARGETS})
target_link_libraries(darknet_sub_node 
    ${catkin_LIBRARIES}
    m
    pthread
    stdc++

  )

#############
## Testing ##
#############

## Add gtest based cpp test target and link libraries
# catkin_add_gtest(${PROJECT_NAME}-test test/test_ros_dnn_test.cpp)
# if(TARGET ${PROJECT_NAME}-test)
#   target_link_libraries(${PROJECT_NAME}-test ${PROJECT_NAME})
# endif()

## Add folders to be run by python nosetests
# catkin_add_nosetests(test)
