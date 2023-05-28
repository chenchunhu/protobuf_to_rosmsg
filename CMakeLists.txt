cmake_minimum_required(VERSION 3.2)
project(rosmsg_converter)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
message(STATUS "C++11 support: OK")

if (NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
    set(CMAKE_BUILD_TYPE "Release")
    set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif ()
set(CMAKE_BUILD_TYPE "Release")
message(STATUS "Build type: ${CMAKE_BUILD_TYPE}")

# ----- Set compiler options ----
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -O0 ")
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -O0")

set(CMAKE_C_FLAGS_RELEASE "${CMAKE_C_FLAGS_RELEASE} -O3 -Wall -Wno-sign-compare")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -O3 -Wall -Wno-sign-compare")

# ----- find package -----
# pcl
find_package(PCL 1.8 REQUIRED
        COMPONENTS
        common
        )
message(STATUS "PCL_INCLUDE_DIRS: ${PCL_INCLUDE_DIRS}")
add_definitions(${PCL_DEFINITIONS})
message(STATUS "PCL_DEFINITIONS: ${PCL_DEFINITIONS}")
#message(STATUS "PCL_COMMON_LIBRARIES: ${PCL_COMMON_LIBRARIES}")
#message(STATUS "PCL_LIBRARIES: ${PCL_LIBRARIES}")

# ros相关
find_package(catkin REQUIRED COMPONENTS
        roscpp
        sensor_msgs
        nav_msgs
        pcl_ros
        tf
        cv_bridge
        rospy
        )
#message(STATUS "catkin_INCLUDE_DIRS: ${catkin_INCLUDE_DIRS}")
#message(STATUS "catkin_LIBRARIES: ${catkin_LIBRARIES}")

# ----- 设置生成.so的路径 -----
set(NODE_LIB_PATH ${CATKIN_DEVEL_PREFIX}/lib/${PROJECT_NAME})
message(STATUS "NODE_LIB_PATH: ${NODE_LIB_PATH}")

catkin_package(
)


# 生成libproto_msg.so
add_subdirectory(proto_msg)

# node
add_executable(${PROJECT_NAME}
        src/rosmsg_converter_node.cpp
        src/rosmsg_converter.cpp
        src/converter.cpp
        )

target_include_directories(${PROJECT_NAME}
        PRIVATE
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )

target_link_libraries(${PROJECT_NAME}
        PRIVATE
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        proto_msg
        )

add_executable(rawmap_converter
        src/rawmap_converter_node.cpp
        src/rawmap_converter.cpp
        src/converter.cpp
        )

target_include_directories(rawmap_converter
        PRIVATE
        include
        ${catkin_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        )

target_link_libraries(rawmap_converter
        PRIVATE
        ${catkin_LIBRARIES}
        ${PCL_LIBRARIES}
        proto_msg
        )


