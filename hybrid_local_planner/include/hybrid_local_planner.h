/** include the libraries you need in your planner here */
/** for robust path planner interface */
#include <ros/ros.h>
#include <costmap_2d/costmap_2d_ros.h>
#include <costmap_2d/costmap_2d.h>
#include <nav_core/base_global_planner.h>
#include <geometry_msgs/PoseStamped.h>
#include <angles/angles.h>
#include <base_local_planner/world_model.h>
#include <base_local_planner/costmap_model.h>


using std::string;

#ifndef HYBRID_LOCAL_PLANNER_CPP
#define HYBRID_LOCAL_PLANNER_CPP

namespace hybrid_planner
{

    class HybridLocalPlanner : public nav_core::BaseGlobalPlanner
    {
    public:
        HybridLocalPlanner();
        HybridLocalPlanner(std::string name, costmap_2d::Costmap2DROS *costmap_ros);

        /** overridden classes from interface nav_core::BaseGlobalPlanner **/
        void initialize(std::string name, costmap_2d::Costmap2DROS *costmap_ros);
        bool makePlan(const geometry_msgs::PoseStamped &start,
                      const geometry_msgs::PoseStamped &goal,
                      std::vector<geometry_msgs::PoseStamped> &plan);

    private:
        costmap_2d::Costmap2DROS *costmap_ros_;
        double step_size_, min_dist_from_robot_;
        costmap_2d::Costmap2D *costmap_;
        base_local_planner::WorldModel *world_model_; ///< @brief The world model that the controller will use
        bool initialized_;
    };
};
#endif