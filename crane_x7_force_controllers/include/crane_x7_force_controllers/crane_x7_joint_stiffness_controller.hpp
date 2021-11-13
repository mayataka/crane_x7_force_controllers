#ifndef CRANE_X7_JOINT_STIFFNESS_CONTROLLER_HPP_
#define CRANE_X7_JOINT_STIFFNESS_CONTROLLER_HPP_

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
#include "crane_x7_force_controllers_msgs/SetJointStiffness.h"


namespace crane_x7_ros {

class CraneX7JointStiffnessController : public controller_interface::Controller<hardware_interface::EffortJointInterface> {
public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Matrix7d = Eigen::Matrix<double, 7, 7>;

  CraneX7JointStiffnessController();
  bool init(hardware_interface::EffortJointInterface* hardware, 
            ros::NodeHandle& node_handler) override;
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

  Vector7d q_, v_, u_, u_max_, q_goal_;
  Matrix7d Kq_, Kv_; // joint stiffness
};

} // namespace crane_x7_ros

#endif // CRANE_X7_JOINT_STIFFNESS_CONTROLLER_HPP_ 