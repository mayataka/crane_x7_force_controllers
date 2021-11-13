#ifndef CRANE_X7_DIFFERENTIAL_IK_HPP_
#define CRANE_X7_DIFFERENTIAL_IK_HPP_

#include <string>
#include <vector>

#include "Eigen/Core"
#include "Eigen/LU"
#include "pinocchio/fwd.hpp"
#include "pinocchio/multibody/model.hpp"
#include "pinocchio/multibody/data.hpp"

#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
#include "sensor_msgs/JointState.h"

#include "crane_x7_force_controllers_msgs/GoalPosition.h"


namespace crane_x7_ros {

class CraneX7DifferentialIK {
public:
  using Vector7d = Eigen::Matrix<double, 7, 1>;
  using Matrix7d = Eigen::Matrix<double, 7, 7>;
  using Matrix67d = Eigen::Matrix<double, 6, 7>;

  CraneX7DifferentialIK(ros::NodeHandle& node_handle);

private:
  void subscribeJointState(const sensor_msgs::JointState& joint_state);
  void publishGoalPosition(const ros::TimerEvent& time_event);

  pinocchio::Model model_;
  pinocchio::Data data_;
  int end_effector_frame_;
  Vector7d q_, v_, q_goal_;
  Matrix7d Wq_, Wv_, H_;
  Eigen::Matrix3d Wx_;
  Matrix67d J_;
  Eigen::LLT<Matrix7d> llt_;
  double dt_;

  ros::NodeHandle node_handle_;
  ros::Subscriber joint_state_subscriber_;
  ros::Publisher goal_position_publisher_;
  crane_x7_force_controllers_msgs::GoalPosition goal_position_;
  ros::Timer timer_;
};

} // namespace crane_x7_ros

#endif // CRANE_X7_DIFFERENTIAL_IK_HPP_ 