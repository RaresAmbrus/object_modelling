cmake_minimum_required(VERSION 2.8.3)
project(rgbd_grabber)

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs cv_bridge image_transport sensor_msgs pcl_ros)
find_package(OpenCV REQUIRED )
find_package(cv_bridge REQUIRED)

set(CMAKE_CXX_FLAGS "-fpermissive -O3 -g -fPIC -std=c++0x")

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

catkin_package(
   INCLUDE_DIRS 
   LIBRARIES 
   CATKIN_DEPENDS roscpp rospy std_msgs
   DEPENDS 
)

include_directories(include
  ${catkin_INCLUDE_DIRS}
)

add_executable(rgbd_grabber src/rgbd_grabber.cpp)

target_link_libraries(rgbd_grabber
  ${catkin_LIBRARIES} ${OpenCV_LIBS} ${PCL_LIBS}
 )


