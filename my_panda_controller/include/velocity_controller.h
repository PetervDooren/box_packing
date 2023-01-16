//
// Created by amigo on 11-1-23.
//

#ifndef BOX_PACKING_VELOCITY_CONTROLLER_H
#define BOX_PACKING_VELOCITY_CONTROLLER_H


#include <array>

#include <Eigen/Core>
#include <Eigen/Dense>

#include <franka/control_types.h>
#include <franka/duration.h>
#include <franka/robot.h>
#include <franka/robot_state.h>
#include <franka/model.h>

#include <franka_hw/franka_model_interface.h>

class VelocityController {
public:
    VelocityController()= default;

    explicit VelocityController(franka_hw::FrankaModelHandle *model_handle)
    {
        robot_model = model_handle;
    }
    std::array<double, 7> controlLaw(const franka::RobotState& robot_state, const ros::Duration& period, const Eigen::Vector3d& desired_velocity);
private:
    // orientation control
    Eigen::Quaterniond orientation_d = Eigen::Quaterniond(0.0, 1.0, 0.0, 0.0); // w, x, y, z
    double rotational_stiffness = 15.0;
    double rotational_damping = 2.0 * sqrt(rotational_stiffness);

    franka_hw::FrankaModelHandle *robot_model = nullptr;
};


#endif //BOX_PACKING_VELOCITY_CONTROLLER_H
