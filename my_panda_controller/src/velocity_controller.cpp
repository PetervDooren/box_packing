//
// Created by amigo on 11-1-23.
//

#include "velocity_controller.h"

std::array<double, 7> VelocityController::controlLaw(const franka::RobotState& robot_state, const ros::Duration& period, const Eigen::Vector3d& desired_velocity)
{
    // aggregate robot state

    // get state variables
    std::array<double, 7> coriolis_array = robot_model->getCoriolis();
    std::array<double, 42> jacobian_array =
            robot_model->getZeroJacobian(franka::Frame::kEndEffector);

    // convert to Eigen
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
    Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
    Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
    Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
    //Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());
    Eigen::Matrix<double, 6, 1> velocity = jacobian * dq;

    // calculate applied force
    Eigen::Matrix<double, 6, 1> force_applied;
    Eigen::Vector3d orientation_error;

    // position velocity control
    double velocity_gain = 50;
    force_applied.head(3) << velocity_gain * (desired_velocity - velocity.head(3));

    // orientation position control
    // "difference" quaternion
    if (orientation_d.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
    }
    // "difference" quaternion
    Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d);
    orientation_error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
    // Transform to base frame
    orientation_error.tail(3) << -transform.linear() * orientation_error.tail(3);
    // compute control
    Eigen::VectorXd tau_task(7), tau_d(7);

    force_applied.tail(3) << -rotational_stiffness * orientation_error - rotational_damping * velocity.tail(3);

    //std::cout << "Force applied: " << force_applied << std::endl;

    // apply the computed force
    tau_task << jacobian.transpose() * force_applied;
    tau_d << tau_task + coriolis;
    std::array<double, 7> tau_d_array{};
    Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
    return tau_d_array;
}
