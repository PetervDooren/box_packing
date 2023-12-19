cmake_minimum_required(VERSION 3.1.3)
project(box_packing)

#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
    cmake_modules
    rospy
    )

if(NOT FRANKA_IS_FOUND)
    find_package(Franka REQUIRED)
endif()

find_package(Eigen3 REQUIRED)

set(THREADS_PREFER_PTHREAD_FLAG ON)
find_package(Threads REQUIRED)

catkin_python_setup()

catkin_package(
#    INCLUDE_DIRS ...
#    LIBRARIES ...
#    CATKIN DEPENDS ...
#    DEPENDS
    )

include_directories(
        include
    ${catkin_INCLUDE_DIRS}
    )

# Visual servoing demo
add_library(examples_common STATIC
    src/box_packing/visual_servoing/examples_common.cpp
  )
target_link_libraries(examples_common PUBLIC Franka::Franka Eigen3::Eigen)
  
add_executable(visservo
    src/box_packing/visual_servoing/visual_servoing_main.cpp
    src/box_packing/visual_servoing/controller.cpp)
target_link_libraries(visservo Franka::Franka Eigen3::Eigen examples_common Threads::Threads)