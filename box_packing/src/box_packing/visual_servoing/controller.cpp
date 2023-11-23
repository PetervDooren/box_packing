#include "controller.h"

franka::Torques ConstraintController::callback(const franka::RobotState& robot_state, franka::Duration duration)
{
      // get state variables
      std::array<double, 7> coriolis_array = model_.coriolis(robot_state);
      std::array<double, 42> jacobian_array =
          model_.zeroJacobian(franka::Frame::kEndEffector, robot_state);
      // convert to Eigen
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> coriolis(coriolis_array.data());
      Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(jacobian_array.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> q(robot_state.q.data());
      Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
      Eigen::Affine3d transform(Eigen::Matrix4d::Map(robot_state.O_T_EE.data()));
      Eigen::Vector3d position(transform.translation());
      Eigen::Quaterniond orientation(transform.linear());
      // compute error to desired equilibrium pose
      // position error
      Eigen::Matrix<double, 6, 1> error;
      error.head(3) << position - position_d_;
      // orientation error
      // "difference" quaternion
      if (orientation_d_.coeffs().dot(orientation.coeffs()) < 0.0) {
        orientation.coeffs() << -orientation.coeffs();
      }
      // "difference" quaternion
      Eigen::Quaterniond error_quaternion(orientation.inverse() * orientation_d_);
      error.tail(3) << error_quaternion.x(), error_quaternion.y(), error_quaternion.z();
      // Transform to base frame
      error.tail(3) << -transform.linear() * error.tail(3);
      // compute control
      Eigen::VectorXd tau_task(7), tau_d(7);
      // Spring damper system with damping ratio=1
      tau_task << jacobian.transpose() * (-stiffness_ * error - damping_ * (jacobian * dq));
      tau_d << tau_task + coriolis;
      std::array<double, 7> tau_d_array{};
      Eigen::VectorXd::Map(&tau_d_array[0], 7) = tau_d;
      return tau_d_array;
};