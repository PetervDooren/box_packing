#include <geometry_msgs/Point.h>

#include "controller.h"

double evaluateConstraint(const Constraint& c, Eigen::Vector3d position)
{
  switch(c.direction){
    case 1: // get rotation about the x axis
      return atan(position.y()/position.z());
      break;
    case 2: // get rotation about the y axis
      return atan(position.x()/position.z());
      break;
    default:
      std::cerr << "invalid constraint direction: " << c.direction << std::endl;
      return 0; // return true to avoid raising a missing contstraint.
  }
}

/**
 * return the direction [x, y, z, Rx, Ry, Rz] in which the constraint will increase.
 * dc/dP. with P the pose of the end effector expressed in world frame.
*/
Eigen::VectorXd getConstraintDirection(const Constraint& c, Eigen::Vector3d position, Eigen::Quaterniond orientation)
{
  Eigen::VectorXd derivative(6);
  switch(c.direction){
    case 1: // get rotation about the x axis
    {
      //value = atan(position.y()/position.z());
      // position component
      Eigen::Vector3d dydPe; // constraint feature function differentiated w.r.t. the pose of the end effector in its own frame.
      //dydPe << 0, 1/position.z(), -position.y()/(position.z()*position.z());
      dydPe << 0, 0, 0;
      derivative.head(3) << orientation.toRotationMatrix().transpose() * dydPe;
      // orientation component
      Eigen::Vector3d dydRe; // the relation between the constraint feature function and the end effector rotational velocity expressed in its own frame.
      dydRe << 1, 0, 0;
      derivative.tail(3) << orientation.toRotationMatrix().transpose() * dydRe;
      break;
    }
    case 2: // get rotation about the y axis
    {
      //value = atan(position.x()/position.z());
      // position component
      Eigen::Vector3d dydPe; // constraint feature function differentiated w.r.t. the pose of the end effector in its own frame.
      //dydPe << 0, 1/position.z(), -position.x()/(position.z()*position.z());
      dydPe << 0, 0, 0;
      derivative.head(3) << orientation.toRotationMatrix().transpose() * dydPe;
      // orientation component
      Eigen::Vector3d dydRe; // the relation between the constraint feature function and the end effector rotational velocity expressed in its own frame.
      dydRe << 0, -1, 0;
      derivative.tail(3) << orientation.toRotationMatrix().transpose() * dydRe;
      break;
    }
    default:
      std::cerr << "invalid constraint direction: " << c.direction << std::endl;
  }
  return derivative;
}

double constraintControl(const Constraint& c, Eigen::Vector3d position)
{
  double K = 5;
  double dy_max = 3;
  double dy_min = 0.2;

  double value = evaluateConstraint(c, position);
  if (value < c.min)
  {
    return std::max(std::min(-K * (c.min - value), dy_max), dy_min); // positive value
  }
  else if (value > c.max)
  {
    return std::min(std::max(-K * (c.max - value), -dy_max), -dy_min); // negative value
  }
  std::cerr << "active constraint " << c.id << " has value " << value << ", inside [" << c.min << ", " << c.max << "]" << std::endl; 
  return 0;
}

visualization_msgs::Marker createMarker(int id, const Constraint& constraint, double constraint_value)
{
  double r = 0.5; // radius in m

  visualization_msgs::Marker marker;
  marker.id = id;
  marker.header.frame_id = "panda_EE";
  marker.header.stamp = ros::Time();
  marker.type = visualization_msgs::Marker::ARROW;
  marker.action = visualization_msgs::Marker::ADD;
  marker.pose.position.x = 0.0;
  marker.pose.position.y = 0.0;
  marker.pose.position.z = 0.0;
  marker.pose.orientation.x = 0.0;
  marker.pose.orientation.y = 0.0;
  marker.pose.orientation.z = 0.0;
  marker.pose.orientation.w = 1.0;

  // define arrow with two points
  geometry_msgs::Point p1;
  p1.x = 0.0;
  p1.y = 0.0;
  p1.z = r;
  marker.points.push_back(p1);

  if (constraint.direction == 1)
  {
    geometry_msgs::Point p2;
    p2.x = 0.0;
    p2.y = r*sin(constraint_value);
    p2.z = r*cos(constraint_value);
    marker.points.push_back(p2);
  }
  else if (constraint.direction == 2)
  {
    geometry_msgs::Point p2;
    p2.x = r*sin(constraint_value);
    p2.y = 0.0;
    p2.z = r*cos(constraint_value);
    marker.points.push_back(p2);
  }
  else
  {
    std::cout << "warning: constraint " << constraint.id << " has unknown direction " << constraint.direction << std::endl;
  }

  marker.scale.x = 0.01; // shaft diameter
  marker.scale.y = 0.03; // head diameter
  marker.scale.z = 0.01; // head length

  if(constraint_value > constraint.min && constraint_value < constraint.max)
  {
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
  }
  else {
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;
  }
  return marker;
}

ConstraintController::ConstraintController(ros::NodeHandle& node_handle, franka_hw::FrankaModelHandle *model_handle)
{
  robot_model = model_handle;

  // within view constraints
  Constraint Rotx;
  Rotx.id = 0;
  Rotx.direction = 1;
  Rotx.min = -0.1;
  Rotx.max = 0.1;
  constraints_.push_back(Rotx);

  Constraint Roty;
  Roty.id = 1;
  Roty.direction = 2;
  Roty.min = -0.1;
  Roty.max = 0.1;
  constraints_.push_back(Roty);

  // hotfix to set the target position 
  position_d_ << 1, 0, 0;

  marker_publisher_ = node_handle.advertise<visualization_msgs::MarkerArray>("constraints_visualization", 5);
}

std::array<double, 7> ConstraintController::callback(const franka::RobotState& robot_state) const
{
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
    Eigen::Vector3d position(transform.translation());
    Eigen::Quaterniond orientation(transform.linear());

    visualization_msgs::MarkerArray marker_array;

    //std::cout << "euler angles: " << orientation.toRotationMatrix().eulerAngles(0, 1, 2) << std::endl;

    // get current vector of end effector to marker/box
    Eigen::Vector3d position_box_ee_w = position_d_ - position; // position of the box with respect to the end effector in world frame
    Eigen::Vector3d position_box_ee_ee = orientation.toRotationMatrix() * position_box_ee_w; // position of the box with respect to the end effector in endeffector frame
    std::cout << "pos in ee: " << position_box_ee_ee.x() << ", " << position_box_ee_ee.y() << ", " << position_box_ee_ee.z() << std::endl;

    std::vector<int> active_constraint_index;
    // evaluate constraints values
    for (int i=0; i<constraints_.size(); i++){
      Constraint constraint = constraints_[i];
      double constraint_value = evaluateConstraint(constraint, position_box_ee_ee);
      std::cout << "c" << constraint.id << ": " << constraint_value << std::endl;
      
      //visualization
      visualization_msgs::Marker marker = createMarker(constraint.id, constraint, constraint_value);
      marker_array.markers.push_back(marker);

      if(constraint_value > constraint.min && constraint_value < constraint.max)
        continue;
      active_constraint_index.push_back(i);
    }

    // visualization
    marker_publisher_.publish(marker_array);

    if (!active_constraint_index.size() > 0)
    {
      std::cout << "no active constraints " << active_constraint_index.size() << std::endl;
      return {0, 0, 0, 0, 0, 0, 0};
    }

    std::cout << "found " << active_constraint_index.size() << " active constraints"<< std::endl;
    // construct interaction matrix
    Eigen::Matrix<double, Eigen::Dynamic, 6> interaction_matrix;
    interaction_matrix.resize(active_constraint_index.size(), 6);
    
    Eigen::VectorXd constraint_velocity_reference;
    constraint_velocity_reference.resize(active_constraint_index.size(), 1);

    for (int i=0; i<active_constraint_index.size(); i++)
    {
      Constraint constraint = constraints_[active_constraint_index[i]];
      Eigen::Matrix<double, 1, 6> derivative = getConstraintDirection(constraint, position_box_ee_ee, orientation);
      interaction_matrix.row(i) << derivative;

      constraint_velocity_reference(i) = constraintControl(constraint, position_box_ee_ee);
    }
    //std::cout << "interaction matrix: " << std:: endl << interaction_matrix << std::endl;
    //std::cout << "constraint velocity reference: " << constraint_velocity_reference << std::endl;

    // solve the pseudo inverse for joint velocities
    Eigen::Matrix<double, Eigen::Dynamic, 7> MJ;
    MJ.resize(active_constraint_index.size(), 7);
    MJ = interaction_matrix * jacobian;

    Eigen::VectorXd dq_d(7);
    auto COD = MJ.completeOrthogonalDecomposition();
    dq_d = COD.solve(constraint_velocity_reference);

    // joint velocity controller
    std::array<double, 7> dq_d_array{};
    Eigen::VectorXd::Map(&dq_d_array[0], 7) = dq_d;
    return dq_d_array;
};
