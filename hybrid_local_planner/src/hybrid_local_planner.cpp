#include "hybrid_local_planner.h"
#include <pluginlib/class_list_macros.h>


PLUGINLIB_EXPORT_CLASS(hybrid_planner::HybridLocalPlanner, nav_core::BaseLocalPlanner)


namespace hybrid_planner {

HybridLocalPlanner::HybridLocalPlanner()
    : costmap_ros_(NULL), tf_(NULL), initialized_(false) {}

HybridLocalPlanner::HybridLocalPlanner(std::string name, tf2_ros::Buffer *tf,
                           costmap_2d::Costmap2DROS *costmap_ros)
    : costmap_ros_(NULL), tf_(NULL), initialized_(false) {
  initialize(name, tf, costmap_ros);
}

HybridLocalPlanner::~HybridLocalPlanner() {}

void HybridLocalPlanner::initialize(std::string name, tf2_ros::Buffer *tf,
                              costmap_2d::Costmap2DROS *costmap_ros) {
  if (!initialized_) {
    initialized_ = true;
    costmap_ros_ = costmap_ros;
    costmap_ = costmap_ros->getCostmap();
    ROS_INFO("Local Costmap has size x: %d, y: %d", costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
  }
}

bool HybridLocalPlanner::setPlan(
    const std::vector<geometry_msgs::PoseStamped> &orig_global_plan) {
  if (!initialized_) {
    ROS_ERROR(
        "The local planner has not been initialized, please call initialize() "
        "before using this planner");
    return false;
  }
  orig_global_plan_ = orig_global_plan;
  return true;
}

bool HybridLocalPlanner::computeVelocityCommands(geometry_msgs::Twist &cmd_vel) {
  if (!initialized_) {
    ROS_ERROR(
        "The local planner has not been initialized, please call initialize() "
        "before using this planner");
    return false;
  }
  return true;
}

bool HybridLocalPlanner::isGoalReached() {
  if (!initialized_) {
    ROS_ERROR(
        "This local planner has not been initialized, please call initialize() "
        "before using this planner");
    return false;
  }

  return false;
}
}  // namespace hybrid_planner