#include <array>
#include <cmath>
#include <functional>
#include <iostream>
#include <Eigen/Dense>
#include <franka/duration.h>
#include <franka/exception.h>
#include <franka/model.h>
#include <franka/robot.h>
#include <vector>


struct Constraint{
    int id;
    int direction;
    double max;
    double min;
};


class ConstraintController{
    public:
    ConstraintController(const franka::Model& model);

    void SetDesiredPosition(Eigen::Vector3d position_d, Eigen::Quaterniond orientation_d)
    {
        position_d_ = position_d;
        orientation_d_ = orientation_d;
    };
    void SetStiffnessDamping(Eigen::MatrixXd stiffness, Eigen::MatrixXd damping)
    {
        stiffness_ = stiffness;
        damping_ = damping;
    };
    
    std::array<double, 7> callback(const franka::RobotState& robot_state) const;

    private:
    const franka::Model& model_;
    // desired position in world frame
    Eigen::Vector3d position_d_; // temp remove later
    Eigen::Quaterniond orientation_d_; // temp remove later
    Eigen::MatrixXd stiffness_;
    Eigen::MatrixXd damping_;

    std::vector<Constraint> constraints_;
};
