cmake_minimum_required(VERSION 3.10)
project(ArucoDetection)

# Zoek de librealsense2-bibliotheek
find_package(realsense2 REQUIRED)

# Zoek de OpenCV-bibliotheek
find_package(OpenCV REQUIRED)
find_package(Eigen3 REQUIRED)

# Voeg de broncode toe aan het uitvoerbare bestand
add_executable(ArucoDetection src/Aruco_pose_estimation_func.cpp)

# Koppel de librealsense2-bibliotheek
target_link_libraries(ArucoDetection PRIVATE realsense2)

# Koppel de OpenCV-bibliotheek
target_link_libraries(ArucoDetection PRIVATE ${OpenCV_LIBS})
target_include_directories(ArucoDetection PRIVATE ${EIGEN3_INCLUDE_DIRS})

