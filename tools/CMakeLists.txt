cmake_minimum_required(VERSION 3.1.3)
project(tools)

#set(CMAKE_CXX_STANDARD 14)
#set(CMAKE_CXX_STANDARD_REQUIRED ON)

find_package(catkin REQUIRED COMPONENTS
        cmake_modules
        roscpp
        )

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