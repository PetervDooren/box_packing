#pragma once

#include <librealsense2/rs.hpp>
#include <vector>

#include <Eigen/Dense>

#include <ros/ros.h>

class ArucoDetector{
    public:
    ArucoDetector()= default;

    explicit ArucoDetector(ros::NodeHandle& node_handle);

    /** 
     * tries to get the pose of a marker in the frame of the camera 
     * @param position
     * @param orientation
     * @return wether or not the marker was found.
    */
    bool getPose(Eigen::Vector3d& position, Eigen::Quaterniond& orientation);

    private:
    rs2::pipeline p;
    int markerId = 43; // #TODO make configurable
    float markerSize = 0.057; // 5,7cm

    bool visualize = true;
    ros::Publisher marker_publisher_;
};
