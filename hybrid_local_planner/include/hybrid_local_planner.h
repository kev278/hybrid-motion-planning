/** include the libraries you need in your planner here */
/** for robust path planner interface */

#ifndef HYBRID_LOCAL_PLANNER_H
#define HYBRID_LOCAL_PLANNER_H
#include <angles/angles.h>
#include <base_local_planner/costmap_model.h>
#include <base_local_planner/world_model.h>
#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_core/base_global_planner.h>
#include <ros/ros.h>
#include <tf2/utils.h>
#include <tf2_ros/buffer.h>
// abstract class from which our plugin inherits
#include <nav_core/base_local_planner.h>
#include <ros/ros.h>

using std::string;


namespace hybrid_planner {

class HybridLocalPlanner : public nav_core::BaseLocalPlanner {
 public:
  HybridLocalPlanner();
  HybridLocalPlanner(std::string name, tf2_ros::Buffer *tf,
                     costmap_2d::Costmap2DROS *costmap_ros);
  // destructer for the wrapper
  ~HybridLocalPlanner();
  /** overridden classes from interface nav_core::BaseGlobalPlanner **/
  void initialize(std::string name, tf2_ros::Buffer *tf,
                  costmap_2d::Costmap2DROS *costmap_ros);
  bool setPlan(const std::vector<geometry_msgs::PoseStamped> &orig_global_plan);
  bool computeVelocityCommands(geometry_msgs::Twist &cmd_vel);

  bool isGoalReached();

 private:
  costmap_2d::Costmap2DROS *costmap_ros_;  ///<@brief pointer to costmap
  tf2_ros::Buffer *tf_;  ///<@brief pointer to Transform Listener
  bool goal_reached_;
  bool initialized_;
  geometry_msgs::PoseStamped current_goal;
  geometry_msgs::PoseStamped current_pose;
  
};
};  // namespace hybrid_planner
#endif