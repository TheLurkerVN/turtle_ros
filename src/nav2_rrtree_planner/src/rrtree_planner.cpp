#include <cmath>
#include <string>
#include <memory>
#include <vector>
#include "nav2_util/node_utils.hpp"

#include "nav2_rrtree_planner/rrtree_planner.hpp"

namespace nav2_rrtree_planner
{

void RRT_Planner::configure(
  const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
  std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros)
{
  planner_ = std::make_unique<rrt_planner::rrt_planner>();
  node_ = parent.lock();
  name_ = name;
  tf_ = tf;
  planner_->costmap_ = costmap_ros->getCostmap();
  global_frame_ = costmap_ros->getGlobalFrameID();
  clock_ = node_->get_clock();
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".loop_count", rclcpp::ParameterValue(
      50000));
  node_->get_parameter(name_ + ".loop_count", loop_count_);
  nav2_util::declare_parameter_if_not_declared(
    node_, name_ + ".branch_length", rclcpp::ParameterValue(
      10));
  node_->get_parameter(name_ + ".branch_length", branch_len_);
  planner_->iterations = loop_count_;
  planner_->branchLen = branch_len_;
}

void RRT_Planner::cleanup()
{
  RCLCPP_INFO(
    node_->get_logger(), "CleaningUp plugin %s of type NavfnPlanner",
    name_.c_str());
  planner_.reset();
}

void RRT_Planner::activate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Activating plugin %s of type NavfnPlanner",
    name_.c_str());
}

void RRT_Planner::deactivate()
{
  RCLCPP_INFO(
    node_->get_logger(), "Deactivating plugin %s of type NavfnPlanner",
    name_.c_str());
}

nav_msgs::msg::Path RRT_Planner::createPlan(
  const geometry_msgs::msg::PoseStamped & start,
  const geometry_msgs::msg::PoseStamped & goal)
{
  nav_msgs::msg::Path global_path;

  // Checking if the goal and start state is in the global frame
  if (start.header.frame_id != global_frame_) {
    RCLCPP_ERROR(
      node_->get_logger(), "Planner will only except start position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  if (goal.header.frame_id != global_frame_) {
    RCLCPP_INFO(
      node_->get_logger(), "Planner will only except goal position from %s frame",
      global_frame_.c_str());
    return global_path;
  }

  

  unsigned int mx_start, my_start, mx_goal, my_goal;

  planner_->costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start);
  planner_->costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal);

  if (planner_->costmap_->getCost(mx_start, my_start) == nav2_costmap_2d::LETHAL_OBSTACLE) {

      return global_path;
  }

  if (mx_start == mx_goal && my_start == my_goal) {
    global_path.header.stamp = clock_->now();
    global_path.header.frame_id = global_frame_;
    geometry_msgs::msg::PoseStamped pose;
    pose.header = global_path.header;
    pose.pose.position.z = 0.0;

    pose.pose = start.pose;

    global_path.poses.push_back(pose);
    return global_path;
  }
  global_path.poses.clear();
  global_path.header.stamp = node_->now();
  global_path.header.frame_id = global_frame_;

  planner_->setStartAndGoal(start, goal);

  RCLCPP_INFO(
    node_->get_logger(), "Start and Goal configured");
  
  
  std::vector<coordsW> path;
  planner_->generatePathROS(path);
  
  std::cout << "Path length " << path.size() << std::endl;
  
  for (int i = 0; i < path.size(); i++)
  {
    geometry_msgs::msg::PoseStamped pose;
    pose.pose.position.x = path[i].x;
    pose.pose.position.y = path[i].y;
    pose.pose.position.z = 0.0;
    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = 0.0;
    pose.pose.orientation.w = 1.0;
    pose.header.stamp = node_->now();
    pose.header.frame_id = global_frame_;
    global_path.poses.push_back(pose);
  }

  return global_path;
}

}

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(nav2_rrtree_planner::RRT_Planner, nav2_core::GlobalPlanner)