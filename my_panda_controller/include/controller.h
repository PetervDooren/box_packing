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
    Eigen::Vector3d getDesiredPosition(){return position_d_;};
    
    std::array<double, 7> callback(const franka::RobotState& robot_state);

    std::vector<Constraint> getconstraints() {return constraints_;};
    std::vector<double> getconstraintValues() {return constraint_values;};
    std::vector<double> getconstraintVelReferences() {return constraint_velocity_reference_log;};
    std::array<double, 7> getDqLog() {return dq_log;};

private:
    //const franka::Model& model_;
    franka_hw::FrankaModelHandle *robot_model = nullptr;
    
    // desired position in world frame
    Eigen::Vector3d position_d_; // temp remove later

    std::vector<Constraint> constraints_;

    // logging
    std::vector<double> constraint_values;
    std::vector<double> constraint_velocity_reference_log;
    std::array<double, 7> dq_log;

    ros::Publisher marker_publisher_;
};

#endif //BOX_PACKING_VELOCITY_CONTROLLER_H