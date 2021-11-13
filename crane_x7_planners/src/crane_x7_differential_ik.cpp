#include "crane_x7_differential_ik.hpp"

#include "pinocchio/parsers/urdf.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics-derivatives.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/frames-derivatives.hpp"

#include "ros/package.h"


namespace crane_x7_ros {

CraneX7DifferentialIK::CraneX7DifferentialIK(ros::NodeHandle& node_handle) 
  : model_(),
    data_(),
    end_effector_frame_(26),
    q_(Eigen::VectorXd::Zero(7)),
    v_(Eigen::VectorXd::Zero(7)),
    q_goal_(Eigen::VectorXd::Zero(7)),
    Wq_(Eigen::MatrixXd::Zero(7, 7)),
    Wv_(Eigen::MatrixXd::Zero(7, 7)), 
    H_(Eigen::MatrixXd::Zero(7, 7)), 
    dt_(0.01),
    node_handle_(node_handle),
    joint_state_subscriber_(
        node_handle.subscribe(
            "/crane_x7/joint_states", 10, 
            &crane_x7_ros::CraneX7DifferentialIK::subscribeJointState, this)),
    goal_position_publisher_(
        node_handle.advertise<crane_x7_force_controllers_msgs::GoalPosition>(
          "/crane_x7/crane_x7_planner/goal_position", 10)),
    timer_(
        node_handle.createTimer(
            ros::Duration(dt_), 
            &crane_x7_ros::CraneX7DifferentialIK::publishGoalPosition, this)) {
  const std::string path_to_urdf 
      = ros::package::getPath("crane_x7_planners") + "/urdf/crane_x7.urdf";
  pinocchio::urdf::buildModel(path_to_urdf, model_);
  data_ = pinocchio::Data(model_);
  q_goal_ << 0., 0.3979, 0., -2.1994, 0.0563, 0.4817, 0.;
}


void CraneX7DifferentialIK::subscribeJointState(const sensor_msgs::JointState& joint_state) {
  // This is for the actual robot
  q_.coeffRef(0) = joint_state.position[3];
  q_.coeffRef(1) = joint_state.position[4];
  q_.coeffRef(2) = joint_state.position[6];
  q_.coeffRef(3) = joint_state.position[5];
  q_.coeffRef(4) = joint_state.position[1];
  q_.coeffRef(5) = joint_state.position[2];
  q_.coeffRef(6) = joint_state.position[7];
  v_.coeffRef(0) = joint_state.velocity[3];
  v_.coeffRef(1) = joint_state.velocity[4];
  v_.coeffRef(2) = joint_state.velocity[6];
  v_.coeffRef(3) = joint_state.velocity[5];
  v_.coeffRef(4) = joint_state.velocity[1];
  v_.coeffRef(5) = joint_state.velocity[2];
  v_.coeffRef(6) = joint_state.velocity[7];

  // // This is for Gazebo simulation
  // for (int i=0; i<7; ++i) {
  //   q_.coeffRef(i) = joint_state.position[i];
  // }
  // for (int i=0; i<7; ++i) {
  //   v_.coeffRef(i) = joint_state.velocity[i];
  // }
}


void CraneX7DifferentialIK::publishGoalPosition(const ros::TimerEvent& time_event) {
  const double t = time_event.current_expected.toSec();
  pinocchio::computeFrameJacobian(model_, data_, q_, end_effector_frame_, 
                                  pinocchio::LOCAL_WORLD_ALIGNED, J_);
  // const auto& J3 = J_.template topRows<3>();
  // H_.noalias() = J3.transpose() * Xq_ * J3 + Wq_;
  // llt_.compute(H_);

  Eigen::Map<Vector7d>(&(goal_position_.q[0])) = q_goal_;
  goal_position_publisher_.publish(goal_position_);
}

} // namespace crane_x7_ros 


int main(int argc, char * argv[]) {
  ros::init(argc, argv, "CraneX7DifferentialIK");
  ros::NodeHandle nh;
  crane_x7_ros::CraneX7DifferentialIK differential_ik(nh);
  ros::spin();
  return 0;
}