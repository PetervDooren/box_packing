// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#pragma once

#include <string>
#include <vector>

#include <controller_interface/multi_interface_controller.h>
#include <franka_hw/franka_model_interface.h>
#include <franka_hw/franka_state_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/node_handle.h>
#include <ros/time.h>
#include <std_srvs/Trigger.h>

#include <mutex>
#include <thread>

#include "controller.h"

// Initialize data fields for the shared memory.
typedef struct {
    std::mutex mutex;
    bool has_data;
    franka::RobotState robot_state;
} sharedmem_robot_state;

typedef struct {
    std::mutex mutex;
    bool has_data;
    std::array<double, 7> dq_d;
} sharedmem_dq_d;

typedef struct {
    std::mutex mutex;
    bool has_data;
    Eigen::Vector3d position;
    Eigen::Quaterniond orientation;
} sharedmem_pose_d;

namespace my_panda_controller {

    class MyController : public controller_interface::MultiInterfaceController<
            franka_hw::FrankaModelInterface,
            hardware_interface::EffortJointInterface,
            franka_hw::FrankaStateInterface> {
    public:
        bool init(hardware_interface::RobotHW* robot_hardware, ros::NodeHandle& node_handle) override;
        void update(const ros::Time&, const ros::Duration& period) override;
        void starting(const ros::Time&) override;
        void stopping(const ros::Time&) override;

    private:
        //hardware_interface::VelocityJointInterface* velocity_joint_interface_;
        std::unique_ptr<franka_hw::FrankaModelHandle> model_handle_;
        std::unique_ptr<franka_hw::FrankaStateHandle> state_handle_;
        std::vector<hardware_interface::JointHandle> joint_handles_;
        ros::Duration elapsed_time_;

        // controller thread
        sharedmem_dq_d sm_dq_d;
        sharedmem_robot_state sm_robot_state;

        ConstraintController controller;
        std::array<double, 7> dq_d = {0, 0, 0, 0, 0, 0, 0}; // zero order hold for dq_d
        std::unique_ptr<std::thread> controller_thread_ptr_;
        bool shutdown_worker; // Trigger to kill the worker thread
        void workerThreadFunc(const float frequency);

        // camera aruco detector thread
        sharedmem_pose_d sm_pose_d;
        std::unique_ptr<std::thread> camera_thread_ptr_;
        void cameraThreadFunc(const float frequency);

        //velocity publisher: needed for contact detection
        ros::Publisher velocity_pub;

        bool active = false;
        ros::ServiceServer trigger_service_;
        bool trigger_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);

        ros::ServiceServer shutdown_service_;
        bool shutdown_callback(std_srvs::TriggerRequest &req, std_srvs::TriggerResponse &res);
    };

}  // namespace my_panda_controller