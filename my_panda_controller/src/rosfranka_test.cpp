// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include "rosfranka_test.h"

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>
#include <geometry_msgs/TwistStamped.h>

namespace my_panda_controller {

    bool MyController::init(hardware_interface::RobotHW* robot_hardware,
                                              ros::NodeHandle& node_handle) {
        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("MyController: Could not get parameter arm_id");
            return false;
        }

        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names)) {
            ROS_ERROR("MyController: Could not parse joint names");
        }
        if (joint_names.size() != 7) {
            ROS_ERROR_STREAM("MyController: Wrong number of joint names, got "
                                     << joint_names.size() << " instead of 7 names!");
            return false;
        }

        auto* model_interface = robot_hardware->get<franka_hw::FrankaModelInterface>();
        if (model_interface == nullptr) {
            ROS_ERROR_STREAM("MyController: Error getting model interface from hardware");
            return false;
        }
        try {
            model_handle_ = std::make_unique<franka_hw::FrankaModelHandle>(
                    model_interface->getHandle(arm_id + "_model"));
        } catch (hardware_interface::HardwareInterfaceException& ex) {
            ROS_ERROR_STREAM(
                    "MyController: Exception getting model handle from interface: " << ex.what());
            return false;
        }

        auto* effort_joint_interface = robot_hardware->get<hardware_interface::EffortJointInterface>();
        if (effort_joint_interface == nullptr) {
            ROS_ERROR_STREAM("MyController: Error getting effort joint interface from hardware");
            return false;
        }
        for (size_t i = 0; i < 7; ++i) {
            try {
                joint_handles_.push_back(effort_joint_interface->getHandle(joint_names[i]));
            } catch (const hardware_interface::HardwareInterfaceException& ex) {
                ROS_ERROR_STREAM("MyController: Exception getting joint handles: " << ex.what());
                return false;
            }
        }

        auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR("MyController: Could not get state interface from hardware");
            return false;
        }

        try {
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(
                    state_interface->getHandle(arm_id + "_robot"));
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM(
                    "MyController: Exception getting state handle: " << e.what());
            return false;
        }

        // configure controller
        int worker_thread_frequency = 100;
        worker_thread_ptr_ = std::make_unique<std::thread>(&MyController::workerThreadFunc, this, worker_thread_frequency);
        controller = ConstraintController(node_handle, model_handle_.get());

        velocity_pub = node_handle.advertise<geometry_msgs::TwistStamped>("measured_velocity", 1);
        trigger_service_ = node_handle.advertiseService("trigger", &MyController::trigger_callback, this);

        return true;
    }

    void MyController::starting(const ros::Time& /* time */) {
        elapsed_time_ = ros::Duration(0.0);
    }

    void MyController::update(const ros::Time& /* time */,
                                                const ros::Duration& period) {
        // calculate and publish ee velocity
        franka::RobotState robot_state = state_handle_->getRobotState();
        Eigen::Map<const Eigen::Matrix<double, 7, 1>> dq(robot_state.dq.data());
        Eigen::Map<const Eigen::Matrix<double, 6, 7>> jacobian(model_handle_->getZeroJacobian(franka::Frame::kEndEffector).data());

        Eigen::Matrix<double, 6, 1> velocity = jacobian * dq;
        geometry_msgs::TwistStamped measured_velocity;
        measured_velocity.header.frame_id = "panda_link0";
        measured_velocity.twist.linear.x = velocity[0];
        measured_velocity.twist.linear.y = velocity[1];
        measured_velocity.twist.linear.z = velocity[2];
        measured_velocity.twist.angular.x = velocity[3];
        measured_velocity.twist.angular.y = velocity[4];
        measured_velocity.twist.angular.z = velocity[5];
        velocity_pub.publish(measured_velocity);

        if (active) {
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
                    dq_d = sm_dq_d.dq_d;
                    sm_dq_d.has_data = false;
                    std::cout << "update dq_d to ";
                    for (int i=0; i<7; i++)
                        std::cout << dq_d[i] << ", ";
                    std::cout << std::endl;
                }
                sm_dq_d.mutex.unlock();
            }

            elapsed_time_ += period;
            //franka::RobotState robot_state = state_handle_->getRobotState();

            //std::array<double, 7> dq_d = controller.callback(robot_state);

            const std::array<double, 7> k_gains = {{50.0, 50.0, 50.0, 50.0, 30.0, 25.0, 15.0}};

            for (int i = 0; i < joint_handles_.size(); i++) {
                double tau_d = k_gains[i] * (dq_d[i] - dq[i]);
                joint_handles_[i].setCommand(tau_d);
            }
        }
        else
        {
            for (auto joint_handle: joint_handles_) {
                joint_handle.setCommand(0.0);
            }
        }
    }

    void MyController::workerThreadFunc(const float frequency)
    {
        //controller = ConstraintController(node_handle, model_handle_.get());

        ros::Rate r(frequency);
        while(!shutdown_)
        {
            if (active) {
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
                    /*std::cout << "constraint thread: dq_d: ";
                    for (int i=0; i< 7; i++)
                        std::cout << dq_d[i];
                    std::cout << std::endl;*/

                    // Try to lock data to avoid read write collisions.
                    if (sm_dq_d.mutex.try_lock()) {
                        sm_dq_d.dq_d = dq_d;
                        sm_dq_d.has_data = true;
                        sm_dq_d.mutex.unlock();
                    }
                    else {
                        std::cout << "could not write dq_d" << std::endl;
                    }
                }
            }
            r.sleep();
        }
    }

    void MyController::stopping(const ros::Time& /*time*/) {
        // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
        // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
        // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
        shutdown_ = true;
        if (worker_thread_ptr_)
            worker_thread_ptr_->join();
    }

    bool MyController::trigger_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res)
    {
        active = !active;
        res.success = true;
        if (active)
            res.message = "MyController active has been set to True";
        else
            res.message = "MyController active has been set to False";
        return true;
    }
}  // namespace my_panda_controllers

PLUGINLIB_EXPORT_CLASS(my_panda_controller::MyController,
                       controller_interface::ControllerBase)
