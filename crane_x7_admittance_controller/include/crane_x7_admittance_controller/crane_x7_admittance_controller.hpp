#ifndef CRANE_X7_ADMITTANCE_CONTROLLER_HPP_
#define CRANE_X7_ADMITTANCE_CONTROLLER_HPP_

#include <string>
#include <vector>

#include "Eigen/Core"
#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"
#include "pinocchio/container/aligned-vector.hpp"
#include "pinocchio/spatial/force.hpp"

#include "ros/ros.h"
#include "controller_interface/controller.h"
#include "hardware_interface/joint_command_interface.h"
#include "std_msgs/Float64.h"

#include "crane_x7_force_controllers_msgs/GoalPosition.h"
#include "crane_x7_force_controllers_msgs/SetEndEffectorStiffness.h"


namespace crane_x7_ros {

class CraneX7AdmittanceController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  using Vector3d = Eigen::Matrix<double, 3, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Matrix3d = Eigen::Matrix<double, 3, 3>;
  using Matrix7d = Eigen::Matrix<double, 7, 7>;
  using Matrix67d = Eigen::Matrix<double, 6, 7>;

  CraneX7AdmittanceController();
  bool init(hardware_interface::EffortJointInterface* hardware, 
            ros::NodeHandle &node_handler) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

private:
  void update(const ros::Time& time, const ros::Duration& period) override;
  void subscribeGoalPosition(const crane_x7_force_controllers_msgs::GoalPosition& goal_position);

  void subscribeJoint1Current(const std_msgs::Float64& current);
  void subscribeJoint2Current(const std_msgs::Float64& current);
  void subscribeJoint3Current(const std_msgs::Float64& current);
  void subscribeJoint4Current(const std_msgs::Float64& current);
  void subscribeJoint5Current(const std_msgs::Float64& current);
  void subscribeJoint6Current(const std_msgs::Float64& current);
  void subscribeJoint7Current(const std_msgs::Float64& current);

  std::vector<hardware_interface::JointHandle> joint_handlers_;
  ros::Subscriber goal_position_subscriber_, joint1_current_subscriber_,
                  joint2_current_subscriber_, joint3_current_subscriber_,
                  joint4_current_subscriber_, joint5_current_subscriber_,
                  joint6_current_subscriber_, joint7_current_subscriber_;

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::container::aligned_vector<pinocchio::Force> fjoint_;
  int end_effector_frame_;
  double current_unit_, effort_const_;

  static double dxl_current_to_effort(const double current) {
    constexpr double current_unit = 2.69;
    constexpr double effort_const = 1.79;
    return current_unit * current * effort_const * 0.001;
  }

  Vector7d q_, v_, qi_, qi_max_, effort_, effort_bias_, ext_effort_, a_, u_, u_max_, q_goal_;
  Matrix7d Kq_, Kv_, Ki_, Kq0_, Kv0_, Ki0_; // joint stiffness
  Vector3d f_, xq_, xv_, xa_;
  Matrix3d Minv_, B_; // virtual mass-spring stiffness
  Matrix67d J_, dJ_;
};

} // namespace crane_x7_ros

#endif // CRANE_X7_ADMITTANCE_CONTROLLER_HPP_ 