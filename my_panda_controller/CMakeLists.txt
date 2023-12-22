cmake_minimum_required(VERSION 3.1.3)
project(my_panda_controller)

#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    roscpp
    kdl_parser
    orocos_kdl
    visualization_msgs
    franka_hw
    )

find_package(Eigen3 REQUIRED NO_MODULE)
find_package(Franka REQUIRED)
find_package(OpenCV REQUIRED)
find_package(realsense2 REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

catkin_package(
#    INCLUDE_DIRS ...
#    LIBRARIES ...
#    CATKIN DEPENDS ...
#    DEPENDS
    )

include_directories(
        include
    ${catkin_INCLUDE_DIRS}
    ${Eigen_INCLUDE_DIRS}
    ${OpenCV_INCLUDE_DIRS}
    )

add_library(my_controller_plugins
        src/rosfranka_test.cpp
        src/controller.cpp
        src/data_saver.cpp
        src/aruco_detector.cpp)

target_link_libraries(my_controller_plugins PUBLIC
        Franka::Franka
        Threads::Threads
        ${OpenCV_LIBS}
        ${catkin_LIBRARIES}
        realsense2
        )