#ifndef BOX_PACKING_VELOCITY_CONTROLLER_H
#define BOX_PACKING_VELOCITY_CONTROLLER_H

#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <Eigen/Dense>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <franka_hw/franka_model_interface.h>
#include <vector>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>


struct Constraint{
    int id;
    int direction;
    double max;
    double min;
};


class ConstraintController{
public:
    ConstraintController()= default;

    explicit ConstraintController(ros::NodeHandle& node_handle, franka_hw::FrankaModelHandle *model_handle);

    void SetDesiredPosition(Eigen::Vector3d position_d)
    {
        position_d_ = position_d;
    };
    
    std::array<double, 7> callback(const franka::RobotState& robot_state) const;

private:
    //const franka::Model& model_;
    franka_hw::FrankaModelHandle *robot_model = nullptr;
    
    // desired position in world frame
    Eigen::Vector3d position_d_; // temp remove later

    std::vector<Constraint> constraints_;

    ros::Publisher marker_publisher_;
};

#endif //BOX_PACKING_VELOCITY_CONTROLLER_H