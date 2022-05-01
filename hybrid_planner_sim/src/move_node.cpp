#include <custom_srvs/VelCmd.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <ros/ros.h>

#include <cmath>
#include <iostream>


class RobotDriver {
 private:
  //! The node handle we'll be using
  ros::NodeHandle nh_;
  //! We will be publishing to the "/base_controller/command" topic to issue
  //! commands
  ros::Publisher cmd_vel_pub_;
  ros::Subscriber sub;
  std::vector<std::vector<float> > goal;
  int i, j;
  float x, y, x_start, y_start;
  float start_error, error_sum;

 public:
  //! ROS node initialization
  RobotDriver(ros::NodeHandle &nh, std::vector<std::vector<float> > &v) {
    nh_ = nh;
    this->i = 1;
    this->goal = v;
    this->x = goal[0][0];
    this->y = goal[0][1];
    this->x_start = this->y_start = 0;
    ROS_WARN("POS IS %lf %lf", v[0][0], v[0][1]);
    // set up the publisher for the cmd_vel topic
    cmd_vel_pub_ = nh_.advertise<geometry_msgs::Twist>("/cmd_vel", 0.5);
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
    sub = nh_.subscribe("/odom", 1000, &RobotDriver::chatterCallback, this);
  }

  void chatterCallback(const nav_msgs::Odometry::ConstPtr &msg) {
    // ROS_INFO("Vel_X: [%f]  and
    // Vel_Y:[%f]",msg->twist.twist.linear.x,msg->twist.twist.linear.y); //
    // gives linear velocity
    ROS_INFO("X: [%f]  and Y:[%f] and Goal:{X:%f,Y:%f}",
             msg->pose.pose.position.x, msg->pose.pose.position.y, x,
             y);  // gives pose

    float x_curr = msg->pose.pose.position.x;
    float y_curr = msg->pose.pose.position.y;

    float dist =
        sqrt((x - x_curr) * (x - x_curr) + (y - y_curr) * (y - y_curr));

    float error_change = (dist - start_error);

    // ROS_INFO("Error_X: [%f]  and Error_Y:[%f]",error_x,error_y); // gives
    // pose

    ROS_INFO("Dist: [%f]", dist);

    error_sum += dist;

    ROS_INFO("Error Sum: [%f]", error_sum);

    ROS_INFO("Error Change: [%f]", error_change);

    ros::Rate r(100);

    // We will be sending commands of type "twist"
    geometry_msgs::Twist base_cmd;
    base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0.0;

    base_cmd.linear.x = 0.2 * dist + 0.03 * error_sum + 0.03 * error_change;

    start_error = dist;

    float te = acos(msg->pose.pose.orientation.w) * 2;

    if (y < 0) {
      if (-1 * atan2(y - y_start, x - x_start) * (180 / 3.14) < 0) {
        base_cmd.angular.z =
            -0.025 * ((180 - atan2(y - y_start, x - x_start) * (180 / 3.14)) -
                      te * (180 / 3.14));
      } else {
        base_cmd.angular.z =
            -0.025 * (-atan2(y - y_start, x - x_start) * (180 / 3.14) -
                      te * (180 / 3.14));
      }

      ROS_INFO("Goal Theta: [%f]",
               -atan2(y - y_start, x - x_start) * (180 / 3.14));  // gives pose
    } else if (y > 0) {
      if (atan2(y - y_start, x - x_start) * (180 / 3.14) < 0) {
        base_cmd.angular.z =
            0.0035 * (abs(atan2(y - y_start, x - x_start) * (180 / 3.14)) -
                      te * (180 / 3.14)) +
            0.0035 * error_sum + 0.0035 * error_change;
        base_cmd.linear.z =
            (abs(atan2(y - y_start, x - x_start) * (180 / 3.14)) -
             te * (180 / 3.14));
        ROS_INFO("Goal Theta: [%f]",
                 abs(atan2(y - y_start, x - x_start) * (180 / 3.14)));
        ROS_INFO("x_start:[%f] y_start:[%f]", x_start, y_start);
      } else {
        base_cmd.angular.z =
            0.005 * (atan2(y - y_start, x - x_start) * (180 / 3.14) -
                     te * (180 / 3.14)) +
            0.0005 * error_sum + 0.005 * error_change;
        base_cmd.linear.z = (atan2(y - y_start, x - x_start) * (180 / 3.14)) -
                            te * (180 / 3.14);
        ROS_INFO("Goal Theta: [%f], x_start:[%f] y_start:[%f]",
                 atan2(y - y_start, x - x_start) * (180 / 3.14), x_start,
                 y_start);
      }

    } else {
      if (x <= 0) {
        base_cmd.angular.z = 0.025 * (179 - te * (180 / 3.14));
        ROS_INFO("Goal Theta: [%f] and ang_z:[%f]", 180.0,
                 base_cmd.angular.z);  // gives pose
      }
    }

    // stoping condition
    if (dist < 0.15 && i < 4) {
      i++;

      base_cmd.linear.x = base_cmd.linear.y = base_cmd.angular.z = 0;
      x = goal[i][0];
      y = goal[i][1];
      x_start = x_curr;
      y_start = y_curr;
    }

    base_cmd.linear.y = dist;
    ROS_INFO("Theta: [%f] and i:[%d] and l_y:[%f]", te * 180 / 3.14, i,
             base_cmd.linear.y);

    cmd_vel_pub_.publish(base_cmd);
    r.sleep();
  }
};
class moveRobot{
  public:
ros::NodeHandle nh;
  bool move_cb(custom_srvs::VelCmd::Request &req,
             custom_srvs::VelCmd::Response &res) {
  std::vector<std::vector<float>> pos;
  for (int i = 0; i < req.path.poses.size(); i++){
    std::vector<float> pos_ = {req.path.poses[i].pose.position.x, req.path.poses[i].pose.position.y };
    pos.push_back(pos_);

  }
  RobotDriver driver(nh, pos);
  ROS_WARN("HERE");
}

};
int main(int argc, char **argv) {
  // init the ROS node
  ros::init(argc, argv, "test");
  ros::NodeHandle nh;
  moveRobot mv;
  mv.nh = nh;
  ros::ServiceServer service = nh.advertiseService("move_robot",  &moveRobot::move_cb, &mv);
  ros::spin();
}