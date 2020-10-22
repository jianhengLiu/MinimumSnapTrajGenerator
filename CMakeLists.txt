cmake_minimum_required(VERSION 2.8.3)
project(MinimumSnapTrajGenerator)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        geometry_msgs
        nav_msgs
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


set(THIRD_PARTY_LIBS
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

include_directories(
         include
        ${catkin_INCLUDE_DIRS}
)

add_library(trajectoryGenerator
        include/trajGenerator/trajectory_generator_waypoint.h
        src/trajGenerator/trajectory_generator_waypoint.cpp)
target_link_libraries(trajectoryGenerator
        ${catkin_LIBRARIES})

add_executable(trajGenerator_node
        App/trajGenerator_node.cpp
        )
target_link_libraries(trajGenerator_node
        ${THIRD_PARTY_LIBS}
        trajectoryGenerator
        )

