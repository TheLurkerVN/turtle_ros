#ifndef NAV2_RRTREE_PLANNER__RRTREE_PLANNER_HPP_
#define NAV2_RRTREE_PLANNER__RRTREE_PLANNER_HPP_

#include <string>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

#include "nav2_core/global_planner.hpp"
#include "nav_msgs/msg/path.hpp"
#include "nav2_util/robot_utils.hpp"
#include "nav2_util/lifecycle_node.hpp"
#include "nav2_costmap_2d/costmap_2d_ros.hpp"

#include "nav2_rrtree_planner/rrt_planner.hpp"

namespace nav2_rrtree_planner
{

class RRT_Planner : public nav2_core::GlobalPlanner
{
public:
  RRT_Planner() = default;
  ~RRT_Planner() = default;

  // plugin configure
  void configure(
    const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override;

  // plugin cleanup
  void cleanup() override;

  // plugin activate
  void activate() override;

  // plugin deactivate
  void deactivate() override;

  // This method creates path for given start and goal pose.
  nav_msgs::msg::Path createPlan(
    const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal) override;

protected:
  // TF buffer
  std::shared_ptr<tf2_ros::Buffer> tf_;

  // node ptr
  nav2_util::LifecycleNode::SharedPtr node_;

  // Global Costmap
  nav2_costmap_2d::Costmap2D * costmap_;
  std::unique_ptr<rrt_planner::rrt_planner> planner_;
  // The global frame of the costmap
  std::string global_frame_, name_;

  rclcpp::Clock::SharedPtr clock_;
  
  int loop_count_;
  int branch_len_;
};

}

#endif