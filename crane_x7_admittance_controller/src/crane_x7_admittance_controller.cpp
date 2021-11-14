#include "crane_x7_admittance_controller.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"
#include "pinocchio/algorithm/rnea.hpp"

#include "ros/package.h"
#include "pluginlib/class_list_macros.h"


namespace crane_x7_ros {

CraneX7AdmittanceController::CraneX7AdmittanceController()
  : model_(),
    data_(),
    fjoint_(),
    end_effector_frame_(26),
    q_(Vector7d::Zero()),
    v_(Vector7d::Zero()),
    qi_(Vector7d::Zero()),
    qi_max_(Vector7d::Zero()),
    effort_(Vector7d::Zero()),
    u_(Vector7d::Zero()),
    u_max_(Vector7d::Zero()),
    q_goal_(Vector7d::Zero()),
    Kq_(Matrix7d::Zero()),
    Kv_(Matrix7d::Zero()),
    Ki_(Matrix7d::Zero()),
    Kq0_(Matrix7d::Zero()),
    Kv0_(Matrix7d::Zero()),
    Ki0_(Matrix7d::Zero()),
    Kqee_(Matrix3d::Zero()),
    Kvee_(Matrix3d::Zero()),
    Kiee_(Matrix3d::Zero()),
    J_(Matrix67d::Zero()) {
  const std::string path_to_urdf 
      = ros::package::getPath("crane_x7_compliance_controller") + "/urdf/crane_x7.urdf";
  pinocchio::urdf::buildModel(path_to_urdf, model_);
  data_ = pinocchio::Data(model_);
  fjoint_ = pinocchio::container::aligned_vector<pinocchio::Force>(
                model_.joints.size(), pinocchio::Force::Zero());
  u_max_ << 10.0, 4.0, 4.0, 4.0, 4.0, 4.0, 4.0; // This is from URDF
  q_goal_ << 0., 0.3979, 0., -2.1994, 0.0563, 0.4817, 0.;
  Kqee_ = 100.0 * Matrix3d::Identity(); // P
  Kvee_ = 50.0 * Matrix3d::Identity(); // D
  Kiee_ = 1.0 * Matrix3d::Identity(); // I
  Kq0_ = 0.0 * Matrix7d::Identity(); // P
  Kv0_ = 0.1 * Matrix7d::Identity(); // D
  Ki0_ = 0.0  * Matrix7d::Identity(); // I
  qi_max_ << 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0; // anti-windup
}


bool CraneX7AdmittanceController::init(
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
      &crane_x7_ros::CraneX7AdmittanceController::subscribeGoalPosition, this);
  joint1_current_subscriber_ = node_handle.subscribe(
      joint_names[0]+"/current", 10, 
      &crane_x7_ros::CraneX7AdmittanceController::subscribeJoint1Current, this);
  joint2_current_subscriber_ = node_handle.subscribe(
      joint_names[1]+"/current", 10, 
      &crane_x7_ros::CraneX7AdmittanceController::subscribeJoint2Current, this);
  joint3_current_subscriber_ = node_handle.subscribe(
      joint_names[2]+"/current", 10, 
      &crane_x7_ros::CraneX7AdmittanceController::subscribeJoint3Current, this);
  joint4_current_subscriber_ = node_handle.subscribe(
      joint_names[3]+"/current", 10, 
      &crane_x7_ros::CraneX7AdmittanceController::subscribeJoint4Current, this);
  joint5_current_subscriber_ = node_handle.subscribe(
      joint_names[4]+"/current", 10, 
      &crane_x7_ros::CraneX7AdmittanceController::subscribeJoint5Current, this);
  joint6_current_subscriber_ = node_handle.subscribe(
      joint_names[5]+"/current", 10, 
      &crane_x7_ros::CraneX7AdmittanceController::subscribeJoint6Current, this);
  joint7_current_subscriber_ = node_handle.subscribe(
      joint_names[6]+"/current", 10, 
      &crane_x7_ros::CraneX7AdmittanceController::subscribeJoint7Current, this);
  return true;
}


void CraneX7AdmittanceController::starting(const ros::Time& time) {
}


void CraneX7AdmittanceController::stopping(const ros::Time& time) {
}


void CraneX7AdmittanceController::update(const ros::Time& time, 
                                         const ros::Duration& period) { 
  std::cout << "effort estimation = " << effort_.transpose() << std::endl;
  for (int i=0; i<7; ++i) {
    q_.coeffRef(i) = joint_handlers_[i].getPosition();
    v_.coeffRef(i) = joint_handlers_[i].getVelocity();
  }
  qi_.noalias() +=  (q_ - q_goal_);
  // anti-windup
  for (int i=0; i<7; ++i) {
    if (qi_.coeff(i) > qi_max_.coeff(i)) {
      qi_.coeffRef(i) = qi_max_.coeff(i);
    }
    else if (qi_.coeff(i) < - qi_max_.coeff(i)) {
      qi_.coeffRef(i) = - qi_max_.coeff(i);
    }
  }
  pinocchio::computeFrameJacobian(model_, data_, q_, end_effector_frame_, 
                                  pinocchio::LOCAL_WORLD_ALIGNED, J_);
  const auto& J3 = J_.template topRows<3>();
  Kq_.noalias() = Kq0_ + J3.transpose() * Kqee_ * J3;
  Kv_.noalias() = Kv0_ + J3.transpose() * Kvee_ * J3;
  Ki_.noalias() = Ki0_ + J3.transpose() * Kiee_ * J3;
  pinocchio::computeStaticTorque(model_, data_, q_, fjoint_);
  u_ = data_.tau; // gravity compensation
  u_.noalias() -= Kq_ * (q_ - q_goal_); // P
  u_.noalias() -= Kv_ * v_; // D
  u_.noalias() -= Ki_ * qi_; // I
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


void CraneX7AdmittanceController::subscribeGoalPosition(
    const crane_x7_force_controllers_msgs::GoalPosition& goal_position) {
  q_goal_ = Eigen::Map<const Vector7d>(&goal_position.q[0]);
}


void CraneX7AdmittanceController::subscribeJoint1Current(
    const std_msgs::Float64& current) {
  effort_.coeffRef(0) = dxl_current_to_effort(current.data);
}

void CraneX7AdmittanceController::subscribeJoint2Current(
    const std_msgs::Float64& current) {
  effort_.coeffRef(1) = dxl_current_to_effort(current.data);
}

void CraneX7AdmittanceController::subscribeJoint3Current(
    const std_msgs::Float64& current) {
  effort_.coeffRef(2) = dxl_current_to_effort(current.data);
}

void CraneX7AdmittanceController::subscribeJoint4Current(
    const std_msgs::Float64& current) {
  effort_.coeffRef(3) = dxl_current_to_effort(current.data);
}

void CraneX7AdmittanceController::subscribeJoint5Current(
    const std_msgs::Float64& current) {
  effort_.coeffRef(4) = dxl_current_to_effort(current.data);
}

void CraneX7AdmittanceController::subscribeJoint6Current(
    const std_msgs::Float64& current) {
  effort_.coeffRef(5) = dxl_current_to_effort(current.data);
}

void CraneX7AdmittanceController::subscribeJoint7Current(
    const std_msgs::Float64& current) {
  effort_.coeffRef(6) = dxl_current_to_effort(current.data);
}

} // namespace crane_x7_ros


PLUGINLIB_EXPORT_CLASS(crane_x7_ros::CraneX7AdmittanceController, 
                       controller_interface::ControllerBase)