#ifndef CRANE_X7_COMPLIANCE_CONTROLLER_HPP_
#define CRANE_X7_COMPLIANCE_CONTROLLER_HPP_

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

#include "crane_x7_force_controllers_msgs/GoalPosition.h"
#include "crane_x7_force_controllers_msgs/SetEndEffectorStiffness.h"


namespace crane_x7_ros {

class CraneX7ComplianceController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  using Vector3d = Eigen::Matrix<double, 3, 1>;
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Matrix3d = Eigen::Matrix<double, 3, 3>;
  using Matrix7d = Eigen::Matrix<double, 7, 7>;
  using Matrix67d = Eigen::Matrix<double, 6, 7>;

  CraneX7ComplianceController();
  bool init(hardware_interface::EffortJointInterface* hardware, 
            ros::NodeHandle &node_handler) override;
  void starting(const ros::Time& time) override;
  void stopping(const ros::Time& time) override;

private:
  void update(const ros::Time& time, const ros::Duration& period) override;
  void subscribeGoalPosition(const crane_x7_force_controllers_msgs::GoalPosition& goal_position);

  std::vector<hardware_interface::JointHandle> joint_handlers_;
  ros::Subscriber goal_position_subscriber_;

  pinocchio::Model model_;
  pinocchio::Data data_;
  pinocchio::container::aligned_vector<pinocchio::Force> fjoint_;
  int end_effector_frame_;

  Vector7d q_, v_, qi_, qi_max_, u_, u_max_, q_goal_;
  Matrix7d Kq_, Kv_, Ki_, Kq0_, Kv0_, Ki0_; // joint stiffness
  Matrix3d Kqee_, Kvee_, Kiee_; // end-effector stiffness
  Matrix67d J_;
};

} // namespace crane_x7_ros

#endif // CRANE_X7_COMPLIANCE_CONTROLLER_HPP_ 