#include "crane_x7_inverse_dynamics_controller.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include "ros/package.h"
#include "pluginlib/class_list_macros.h"


namespace crane_x7_ros {

CraneX7InverseDynamicsController::CraneX7InverseDynamicsController()
  : model_(),
    data_(),
    fjoint_(),
    q_(Vector7d::Zero()), 
    v_(Vector7d::Zero()), 
    a_(Vector7d::Zero()), 
    qi_(Vector7d::Zero()),
    qi_max_(Vector7d::Zero()),
    u_(Vector7d::Zero()),
    u_max_(Vector7d::Zero()),
    q_goal_(Vector7d::Zero()) {
  const std::string path_to_urdf 
      = ros::package::getPath("crane_x7_inverse_dynamics_controller") + "/urdf/crane_x7.urdf";
  pinocchio::urdf::buildModel(path_to_urdf, model_);
  data_ = pinocchio::Data(model_);
  fjoint_ = pinocchio::container::aligned_vector<pinocchio::Force>(
                model_.joints.size(), pinocchio::Force::Zero());
  u_max_ << 10.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0; // This is from URDF
  q_goal_ << 0., 0.3979, 0., -2.1994, 0.0563, 0.4817, 0.;
  Kq_ = 100.0 * Matrix7d::Identity(); // P
  Kv_ = 25.0 * Matrix7d::Identity(); // D
  Ki_ = 10.0 * Matrix7d::Identity(); // I
  qi_max_ << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0; // anti-windup
}


bool CraneX7InverseDynamicsController::init(
    hardware_interface::EffortJointInterface* hardware, 
    ros::NodeHandle& node_handle) {
  const std::vector<std::string> joint_names = {
      "crane_x7_shoulder_fixed_part_pan_joint",
      "crane_x7_shoulder_revolute_part_tilt_joint",
      "crane_x7_upper_arm_revolute_part_twist_joint",
      "crane_x7_upper_arm_revolute_part_rotate_joint",
      "crane_x7_lower_arm_fixed_part_joint",
      "crane_x7_lower_arm_revolute_part_joint",
      "crane_x7_wrist_joint"};
  if (!joint_handlers_.empty()) {
    return false;
  }
  for (const auto& joint_name : joint_names) {
    joint_handlers_.push_back(hardware->getHandle(joint_name));
  }
  goal_position_subscriber_ = node_handle.subscribe(
      "/crane_x7/crane_x7_planner/goal_position", 10, 
      &crane_x7_ros::CraneX7InverseDynamicsController::subscribeGoalPosition, this);
  return true;
}


void CraneX7InverseDynamicsController::starting(const ros::Time& time) {
}


void CraneX7InverseDynamicsController::stopping(const ros::Time& time) {
}


void CraneX7InverseDynamicsController::update(const ros::Time& time, 
                                              const ros::Duration& period) { 
  for (int i=0; i<7; ++i) {
    q_.coeffRef(i) = joint_handlers_[i].getPosition();
    v_.coeffRef(i) = joint_handlers_[i].getVelocity();
  }
  qi_.noalias() += (q_ - q_goal_);
  // anti-windup
  for (int i=0; i<7; ++i) {
    if (qi_.coeff(i) > qi_max_.coeff(i)) {
      qi_.coeffRef(i) = qi_max_.coeff(i);
    }
    else if (qi_.coeff(i) < - qi_max_.coeff(i)) {
      qi_.coeffRef(i) = - qi_max_.coeff(i);
    }
  }
  a_.noalias()  = - Kq_ * (q_ - q_goal_); // P
  a_.noalias() -= Kv_ * v_; // D
  a_.noalias() -= Ki_ * qi_; // I
  u_ = pinocchio::rnea(model_, data_, q_, v_, a_, fjoint_);
  // saturation
  for (int i=0; i<7; ++i) {
    if (u_.coeff(i) > u_max_.coeff(i)) {
      u_.coeffRef(i) = u_max_.coeff(i);
    }
    else if (u_.coeff(i) < - u_max_.coeff(i)) {
      u_.coeffRef(i) = - u_max_.coeff(i);
    }
  }
  for (int i=0; i<7; ++i) {
    joint_handlers_[i].setCommand(u_.coeff(i));
  }
}


void CraneX7InverseDynamicsController::subscribeGoalPosition(
    const crane_x7_force_controllers_msgs::GoalPosition& goal_position) {
  q_goal_ = Eigen::Map<const Vector7d>(&goal_position.q[0]);
}

} // namespace crane_x7_ros


PLUGINLIB_EXPORT_CLASS(crane_x7_ros::CraneX7InverseDynamicsController, 
                       controller_interface::ControllerBase)