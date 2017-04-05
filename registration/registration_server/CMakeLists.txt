cmake_minimum_required(VERSION 2.8.3)
project(registration_server)

set(CMAKE_CXX_FLAGS "-O4 -fPIC -std=c++0x -fpermissive ${CMAKE_CXX_FLAGS}")

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs geometry_msgs sensor_msgs registration_services tf pcl_ros qt_build siftgpu cv_bridge)


catkin_package(
   INCLUDE_DIRS 
   CATKIN_DEPENDS std_msgs geometry_msgs sensor_msgs registration_services pcl_ros siftgpu cv_bridge
   DEPENDS libceres-dev suitesparse qt_build libqt4-dev
   LIBRARIES rgbd_view_registration_optimizer
)

find_package(PCL 1.7 REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

rosbuild_prepare_qt4(QtCore QtXml)

FIND_PACKAGE(Ceres REQUIRED)
INCLUDE_DIRECTORIES(${CERES_INCLUDE_DIRS})

include_directories(${catkin_INCLUDE_DIRS}
                    include)


add_library(rgbd_view_registration_optimizer include/rgbd_view_registration_server/rgbd_view_registration_optimizer.h
                                               include/rgbd_view_registration_server/rgbd_view_registration_residual.h
                                               include/rgbd_view_registration_server/sift_wrapper.h
                                               src/rgbd_view_registration_optimizer.cpp
                                               src/sift_wrapper.cpp)

add_dependencies(rgbd_view_registration_optimizer registration_services_gencpp)
target_link_libraries(rgbd_view_registration_optimizer ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${QT_LIBRARIES} ${CERES_LIBRARIES})

add_executable(rgbd_view_registration_server src/rgbd_view_registration_main.cpp)
target_link_libraries(rgbd_view_registration_server ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${QT_LIBRARIES} ${CERES_LIBRARIES} rgbd_view_registration_optimizer)
add_dependencies(rgbd_view_registration_server registration_services_gencpp)


add_executable(test_rgbd_view_registration test/test_rgbd_view_registration.cpp)
target_link_libraries(test_rgbd_view_registration ${catkin_LIBRARIES} ${PCL_LIBRARIES} ${QT_LIBRARIES} ${CERES_LIBRARIES} rgbd_view_registration_optimizer)
add_dependencies(test_rgbd_view_registration registration_services_gencpp)

