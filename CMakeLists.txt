cmake_minimum_required(VERSION 2.8.3)
project(visual_tools)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        cv_bridge
        pcl_ros
        geometry_msgs
        nav_msgs
        quadrotor_msgs
        poly_traj_utils
        message_generation
        )

find_package(Eigen3 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(OpenCV 3.2 REQUIRED)
catkin_package(
        INCLUDE_DIRS include
        LIBRARIES simulator
        CATKIN_DEPENDS roscpp rospy std_msgs poly_traj_utils
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${OpenCV_INCLUDE_DIRS}
)
link_directories(${PCL_LIBRARY_DIRS})

set(THIRD_PARTY
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

add_executable(moving_frame_pub
        Apps/moving_frame_pub.cpp
        )
target_link_libraries(moving_frame_pub
        ${THIRD_PARTY}
        )
