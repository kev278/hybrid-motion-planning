#include "hybrid_global_planner.h"

#include <pluginlib/class_list_macros.h>
#include <tf2/convert.h>
#include <tf2/utils.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "nav_msgs/GetPlan.h"
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
    path_pub = private_nh.advertise<nav_msgs::Path>("chatter", 1000);
    ros::NodeHandle n;
    rrt_client = n.serviceClient<nav_msgs::GetPlan>("rrt_get_plan");
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
  unsigned int mx_start, my_start, mx_goal, my_goal;
  costmap_->worldToMap(start.pose.position.x, start.pose.position.y, mx_start, my_start);
  costmap_->worldToMap(goal.pose.position.x, goal.pose.position.y, mx_goal, my_goal);
  nav_msgs::GetPlan srv;
  srv.request.start.pose.position.x = mx_start;
  srv.request.start.pose.position.y = my_start;
  
  srv.request.goal.pose.position.x = mx_goal;
  srv.request.goal.pose.position.y = my_goal;
  if (rrt_client.call(srv))
  {
    ROS_INFO("Got path");
  }
  else
  {
    ROS_ERROR("Failed to call service rrt_planner");
    return 1;
  }
  nav_msgs::Path path = srv.response.plan;
  
  path.header = start.header;
  path.header.frame_id = "map";
  unsigned int mx, my;
  double wx, wy;
  ROS_INFO("PATH POSES ARE %lf", path.poses.size());
  for (int i = 0; i < path.poses.size(); i++){
    mx = path.poses[i].pose.position.x;
    my = path.poses[i].pose.position.y;
    path.poses[i].pose.orientation.w = 1;
    costmap_->mapToWorld(mx, my, wx, wy);
    path.poses[i].pose.position.x = wx;
    path.poses[i].pose.position.y = wy;
    ROS_INFO("HERE %lf %lf %lf %lf", mx, my ,wx, wy);
  }
  ROS_INFO_STREAM("PATH is done, publishing" << path);
  path_pub.publish(path);
  ros::spinOnce();
  }
};
