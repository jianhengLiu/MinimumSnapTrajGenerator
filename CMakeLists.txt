cmake_minimum_required(VERSION 2.8.3)
project(MinimumSnapTrajGenerator)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11 -03)

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11 -O3") #c++11
#set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g -march=native --fast-math -ffast-math -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -march=native -mtune=native -funroll-loops -DNDEBUG -DBOOST_DISABLE_ASSERTS" CACHE STRING COMPILE_FLAGS FORCE)

find_package(catkin REQUIRED COMPONENTS
        roscpp
        std_msgs
        image_transport
        cv_bridge
        geometry_msgs
        nav_msgs
        mavros_msgs
        tf
        )

find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)


set(THIRD_PARTY_LIBS
        ${Sophus_LIBRARIES}
        ${Pangolin_LIBRARIES} GL GLU GLEW glut
        ${GTEST_BOTH_LIBRARIES}
        ${GLOG_LIBRARIES}
        ${GFLAGS_LIBRARIES}
        pthread
        ${CSPARSE_LIBRARY}
        ${catkin_LIBRARIES}
        ${OpenCV_LIBRARIES}
        )

include_directories(
        # include
        ${catkin_INCLUDE_DIRS}
        ${CMAKE_CURRENT_SOURCE_DIR}/include
)
catkin_package(

)

add_library(loadController
        include/loadController/flightController.h
        src/loadController/flightController.cpp)
target_link_libraries(loadController
        ${catkin_LIBRARIES}
        ${THIRD_PARTY_LIBS}
        )
add_library(trajectoryGenerator
        include/trajGenerator/trajectory_generator_waypoint.h
        src/trajGenerator/trajectory_generator_waypoint.cpp)
target_link_libraries(trajectoryGenerator
        ${catkin_LIBRARIES})

add_executable(trajGenerator_node
        App/trajGenerator_node.cpp
        src/loadController/math.cpp
        include/loadController/common_include.h
        include/loadController/math.h
        )
target_link_libraries(trajGenerator_node
        ${THIRD_PARTY_LIBS}
        loadController
        trajectoryGenerator
        )

