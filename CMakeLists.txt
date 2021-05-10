cmake_minimum_required(VERSION 2.8.3)
project(visual_tools)

add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -Wall -g -fPIC")

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        )

find_package(Eigen3 REQUIRED)

catkin_package(
        INCLUDE_DIRS include
        LIBRARIES visual_tools
        CATKIN_DEPENDS roscpp rospy std_msgs 
        #  DEPENDS system_lib
)

include_directories(
        SYSTEM
        include
        ${catkin_INCLUDE_DIRS}
        ${Eigen3_INCLUDE_DIRS}
  
)

set(THIRD_PARTY
        ${catkin_LIBRARIES}
    
        )

add_executable(moving_frame_pub
        Apps/moving_frame_pub.cpp
        )
target_link_libraries(moving_frame_pub
        ${THIRD_PARTY}
        )
