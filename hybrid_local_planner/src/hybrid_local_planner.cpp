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
    char **argc;
    int argv;
    ros::NodeHandle n;
    ros::ServiceClient client = n.serviceClient<custom_srvs::VelCmd>("move_robot");
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
  ROS_INFO("SET PLAN");
  nav_msgs::Path path;
  path.poses = orig_global_plan;
  custom_srvs::VelCmd srv;
  srv.request.path = path;
  client.call(srv);
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
  ROS_INFO("LOCAL PLANNER ");
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