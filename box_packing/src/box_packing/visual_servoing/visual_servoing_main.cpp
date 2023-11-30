// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <Eigen/Dense>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include "examples_common.h"
#include <atomic>
#include <mutex>
#include <thread>

#include "controller.h"

int main(int argc, char** argv) {
  // Check whether the required arguments were passed
  if (argc != 2) {
    std::cerr << "Usage: " << argv[0] << " <robot-hostname>" << std::endl;
    return -1;
  }

  const double constraint_control_rate = 100.0;

  // Compliance parameters
  const double translational_stiffness{150.0};
  const double rotational_stiffness{10.0};
  Eigen::MatrixXd stiffness(6, 6), damping(6, 6);
  stiffness.setZero();
  stiffness.topLeftCorner(3, 3) << translational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  stiffness.bottomRightCorner(3, 3) << rotational_stiffness * Eigen::MatrixXd::Identity(3, 3);
  damping.setZero();
  damping.topLeftCorner(3, 3) << 2.0 * sqrt(translational_stiffness) *
                                     Eigen::MatrixXd::Identity(3, 3);
  damping.bottomRightCorner(3, 3) << 2.0 * sqrt(rotational_stiffness) *
                                         Eigen::MatrixXd::Identity(3, 3);

  // Initialize data fields for the shared memory.
  struct {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
  } sm_robot_state{};
  struct {
    std::mutex mutex;
    bool has_data;
    std::array<double, 7> dq_d;
  } sm_dq_d{};

  std::atomic_bool running{true};
  // forward declare thread
  std::thread constraint_control_thread;
  try {
    // connect to robot
    franka::Robot robot(argv[1]);
    setDefaultBehavior(robot);
    // load the kinematics and dynamics model
    franka::Model model = robot.loadModel();
    franka::RobotState initial_state = robot.readOnce();
    // equilibrium point is the initial position
    Eigen::Affine3d initial_transform(Eigen::Matrix4d::Map(initial_state.O_T_EE.data()));
    //Eigen::Vector3d position_d(initial_transform.translation());
    Eigen::Vector3d position_d;
    position_d << 1, 0, 0;
    Eigen::Quaterniond orientation_d(initial_transform.linear());
    // set collision behavior
    robot.setCollisionBehavior({{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}},
                               {{100.0, 100.0, 100.0, 100.0, 100.0, 100.0}});

    ConstraintController controller(model);
    controller.SetDesiredPosition(position_d, orientation_d);

    // start real-time control loop
    std::cout << "WARNING: Collision thresholds are set to high values. "
              << "Make sure you have the user stop at hand!" << std::endl
              << "After starting try to push the robot and see how it reacts." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();

    // start constraint control thread.
    constraint_control_thread = std::thread([constraint_control_rate, controller, &sm_robot_state, &sm_dq_d, &running]() {
      while (running) {
        // Sleep to achieve the desired print rate.
        std::this_thread::sleep_for(
            std::chrono::milliseconds(static_cast<int>((1.0 / constraint_control_rate * 1000.0))));
          
        franka::RobotState robot_state;
        bool newdata = false;
        // Try to lock data to avoid read write collisions.
        if (sm_robot_state.mutex.try_lock()) {
          if (sm_robot_state.has_data) {
            // read robot state data
            robot_state = sm_robot_state.robot_state;
            sm_robot_state.has_data = false;
            newdata = true;
          }
          sm_robot_state.mutex.unlock();
        }
        if (newdata){
          // calculate new joint velocity reference
          std::array<double, 7> dq_d = controller.callback(robot_state);
          std::cout << "constraint thread: dq_d: ";
          for (int i=0; i< 7; i++)
            std::cout << dq_d[i];
          std::cout << std::endl;

          // Try to lock data to avoid read write collisions.
          if (sm_dq_d.mutex.try_lock()) {
            sm_dq_d.dq_d = dq_d;
            sm_dq_d.has_data = true;
            sm_dq_d.mutex.unlock();
          }
        }
      }
    });

    std::array<double, 7> dq_d_last = {0, 0, 0, 0, 0, 0, 0};
    // define callback for the torque control loop
    std::function<franka::Torques(const franka::RobotState&, franka::Duration)>
        impedance_control_callback = [&](const franka::RobotState& robot_state,
                                         franka::Duration duration) -> franka::Torques {
      if (sm_robot_state.mutex.try_lock()) {
        // read robot state data
        sm_robot_state.robot_state = robot_state;
        sm_robot_state.has_data = true;
        sm_robot_state.mutex.unlock();
      }

      // Try to lock data to avoid read write collisions.
      if (sm_dq_d.mutex.try_lock()) {
        if (sm_dq_d.has_data)
        {
          dq_d_last = sm_dq_d.dq_d;
          sm_dq_d.has_data = false;
        }
        sm_dq_d.mutex.unlock();
      }

      const std::array<double, 7> k_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};

      std::array<double, 7> tau_d_calculated;
      for (size_t i = 0; i < 7; i++) {
        tau_d_calculated[i] = k_gains[i] * (dq_d_last[i] - robot_state.dq[i]);
      }

      //return tau_d_calculated;
      return {0, 0, 0, 0, 0, 0, 0}; // safety first
    };
    
    robot.control(impedance_control_callback);
  } catch (const franka::Exception& ex) {
    running = false;
    // print exception
    std::cout << ex.what() << std::endl;
  }

  if (constraint_control_thread.joinable()) {
    constraint_control_thread.join();
  }
  return 0;
}