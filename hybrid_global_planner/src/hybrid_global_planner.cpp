#include "hybrid_global_planner.h"

#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
//#include "informed_rrt_star.hpp"

// register this planner as a BaseHybridGlobalPlanner plugin
PLUGINLIB_EXPORT_CLASS(hybrid_planner::HybridGlobalPlanner,
                       nav_core::BaseGlobalPlanner)

using namespace std;

// Default Constructor
namespace hybrid_planner {

HybridGlobalPlanner::HybridGlobalPlanner()
    : costmap_ros_(NULL),
      costmap_(NULL),
      world_model_(NULL),
      initialized_(false) {}

HybridGlobalPlanner::HybridGlobalPlanner(std::string name,
                                         costmap_2d::Costmap2DROS *costmap_ros)
    : costmap_ros_(NULL),
      costmap_(NULL),
      world_model_(NULL),
      initialized_(false) {
  initialize(name, costmap_ros);
}

void HybridGlobalPlanner::initialize(std::string name,
                                     costmap_2d::Costmap2DROS *costmap_ros) {
  if (!initialized_) {
    ROS_WARN("Global initialization");
    costmap_ros_ =
        costmap_ros;  // initialize the costmap_ros_ attribute to the parameter.
    costmap_ =
        costmap_ros_->getCostmap();  // get the costmap_ from costmap_ros_

    // initialize other planner parameters
    ros::NodeHandle private_nh("~/" + name);
    private_nh.param("step_size", step_size_, costmap_->getResolution());
    private_nh.param("min_dist_from_robot", min_dist_from_robot_, 0.10);
    world_model_ = new base_local_planner::CostmapModel(*costmap_);
    ROS_INFO("Global Costmap has size x: %d, y: %d",
             costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY());
    initialized_ = true;
  } else
    ROS_WARN(
        "The global planner has already been initialized... doing nothing");
}

bool HybridGlobalPlanner::makePlan(
    const geometry_msgs::PoseStamped &start,
    const geometry_msgs::PoseStamped &goal,
    std::vector<geometry_msgs::PoseStamped> &plan) {
  RRT_Node *start_node = new RRT_Node();
  RRT_Node *goal_node = new RRT_Node();
  unsigned int mx_start, my_start, mx_goal, my_goal;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start);

  start_node->row = mx_start;
  start_node->col = my_start;

costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal);

  goal_node->row = mx_goal;
  goal_node->col = my_goal;
  planner = new RRT(start_node, goal_node, costmap_);
  plan = planner->informed_RRT_star(10000, 20);
  if (plan.size() == 0){   
    return false;
  } else {
    return true;
  }
  }
};
