#pragma once

#include <librealsense2/rs.hpp>
#include <vector>

#include <Eigen/Dense>

class ArucoDetector{
    public:
    ArucoDetector();
    /** 
     * tries to get the pose of a marker in the frame of the camera 
     * @param position
     * @param orientation
     * @return wether or not the marker was found.
    */
    bool getPose(Eigen::Vector3d& position, Eigen::Quaterniond& orientation);

    private:
    rs2::pipeline p;
    int markerId = 42; // #TODO make configurable
    float markerSize = 0.10;

    bool visualize = true;
};
