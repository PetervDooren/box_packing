#pragma once

#include <librealsense2/rs.hpp>
#include <vector>

class ArucoDetector{
    public:
    ArucoDetector();
    bool getPose();

    private:
    rs2::pipeline p;
    int markerId = 42; // #TODO make configurable
    float markerSize = 0.10;
};
