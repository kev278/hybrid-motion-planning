/*
Inputs:
Start position
Cost Map - ros costmap pointer

Outputs:
path - std::vector<geometry_msgs>
*/
#include <nav_msgs/OccupancyGrid.h>
#include <ros/ros.h>

#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <random>
#include <vector>

template <typename T>
std::vector<size_t> sort_indexes(const std::vector<T>& v) {
  // initialize original index locations
  std::vector<size_t> idx(v.size());
  iota(idx.begin(), idx.end(), 0);

  // sort indexes based on comparing values in v
  // using std::stable_sort instead of std::sort
  // to avoid unnecessary index re-orderings
  // when v contains elements of equal values
  stable_sort(idx.begin(), idx.end(),
              [&v](size_t i1, size_t i2) { return v[i1] < v[i2]; });

  return idx;
}

template <typename T>
std::vector<double> linspace(T start_in, T end_in, int num_in) {
  std::vector<double> linspaced;

  double start = static_cast<double>(start_in);
  double end = static_cast<double>(end_in);
  double num = static_cast<double>(num_in);

  if (num == 0) {
    return linspaced;
  }
  if (num == 1) {
    linspaced.push_back(start);
    return linspaced;
  }

  double delta = (end - start) / (num - 1);

  for (int i = 0; i < num - 1; ++i) {
    linspaced.push_back(start + delta * i);
  }
  linspaced.push_back(end);  // I want to ensure that start and end
                             // are exactly the same as the input
  return linspaced;
}

using Eigen::MatrixXd;

struct RRT_Node {
 public:
  double row;
  double col;
  RRT_Node* parent = nullptr;
  double cost;
};

class RRT {
 public:
  RRT(RRT_Node* start_, RRT_Node* goal_,
      costmap_2d::Costmap2D* global_costmap_) {
    // ros::NodeHandle n;
    // ros::Subscriber sub = n.subscribe("nav_msgs/global_costmap/costmap", 1,
    // costmap_callback);
    global_costmap = global_costmap_;
    size_row = global_costmap->getSizeInCellsX();

    size_col = global_costmap->getSizeInCellsY();

    vertices.push_back(start_);
    start = start_;
    goal = goal_;
  }

  // Cost Map
  // Working with Cell indexes
  unsigned int size_row;
  unsigned int size_col;
  RRT_Node* start;
  RRT_Node* goal;
  std::vector<RRT_Node*> vertices;
  bool found{false};
  double min_dis = 1000;
  costmap_2d::Costmap2D* global_costmap;

  double dis(RRT_Node* node1, RRT_Node* node2) {
    double node1_row_meter, node1_col_meter, node2_row_meter, node2_col_meter;
    global_costmap->mapToWorld(node1->row, node1->col, node1_row_meter,
                               node1_col_meter);
    global_costmap->mapToWorld(node2->row, node2->col, node2_row_meter,
                               node2_col_meter);

    return sqrt(pow((node1_row_meter - node2_row_meter), 2) +
                pow((node1_col_meter - node2_col_meter), 2));
  }
    double dis_grid(RRT_Node* node1, RRT_Node* node2) {
    return sqrt(pow((node1->row - node2->row), 2) +
                pow((node1->col - node2->col), 2));
  }
bool isThisPointCollides(double mx, double my) {
  // In case of no costmap loaded
  if (global_costmap == nullptr) {
    // no collision
    return false;
  }


  if ((mx < 0) || (my < 0) || (mx >= global_costmap->getSizeInCellsX()) || (my >= global_costmap->getSizeInCellsY())){
    return true;
  }
  // getCost returns unsigned char
  unsigned int cost = static_cast<int>(global_costmap->getCost(mx, my));

  if (cost > 200){
    return true;
  } 
  return false;
}

bool check_collision(RRT_Node* node1, RRT_Node *node2) {
  // In case of no costmap loaded
  if (global_costmap == nullptr) {
    // there is NO obstacles
    return false;
  }

  float dist = dis_grid(node1, node2);
  if (dist < 0.05) {
    return (isThisPointCollides(node2->row, node2->col)) ? true : false;
  } else {
    int steps_number = static_cast<int>(floor(dist/0.05));
    float theta = atan2(node1->col - node2->col, node1->row - node2->row);
    std::pair<float, float> p_n;
    for (int n = 1; n < steps_number; n++) {
      p_n.first = node1->row + n*0.05*cos(theta);
      p_n.second = node1->col + n*0.05*sin(theta);
      if (isThisPointCollides(p_n.first, p_n.second)){
        return false;
    }
    }
    return false;
  }
}
/*   // RRT_Node 2 is the new point
  bool check_collision(RRT_Node* node1, RRT_Node* node2) {
    // convert map to world
    double node1_row_meter, node1_col_meter, node2_row_meter, node2_col_meter;
    global_costmap->mapToWorld(node1->row, node1->col, node1_row_meter,
                               node1_col_meter);
    global_costmap->mapToWorld(node2->row, node2->col, node2_row_meter,
                               node2_col_meter);
    std::vector<double> points_between_row =
        linspace(node1_row_meter, node2_row_meter, 50);
    std::vector<double> points_between_col =
        linspace(node1_col_meter, node2_col_meter, 50);
        unsigned int mx;
        unsigned int my;
    for (int i = 0; i < points_between_row.size(); i++) {
      for (int j = 0; j < points_between_col.size(); j++) {
        // convert world to map and check if point
        double wx = points_between_row[i];
        double wy = points_between_col[j];
        global_costmap->worldToMap(wx, wy, mx, my);
        unsigned char c = global_costmap->getCost(mx, my);
        int cost = c;

        if (cost > 100) {
          ROS_INFO("COLLISION :(, %lf %lf %lf %lf %lf %lf %d", node1->row, node1->col, node2->row, node2->col, mx, my, cost);
          return true;
        } else{
          ROS_WARN("NO COLLISION %lf %lf %lf %lf %d", node1->row, node1->col, node2->row, node2->col, cost);
        }
      }
    }
    return false;
  } */

  std::pair<double, double> get_new_point(double goal_bias) {
    // Generate a pair of random numbers TODO
    std::random_device rd;   // obtain a random number from hardware
    std::mt19937 gen(rd());  // seed the generator
    std::uniform_int_distribution<> x0(0, 100);  // define the range
    double point_ = x0(gen);
    point_ /= 100;

    if (point_ < goal_bias) {
      return std::pair<double, double>{goal->row, goal->col};
    }

    else {
      std::uniform_int_distribution<> x(0, size_row - 1),
          y(0, size_col - 1);  // define the range
      double point_x = x(gen);
      double point_y = y(gen);  // generate numbers
      return std::pair<double, double>{point_x, point_y};
    }
  }

  std::pair<double, double> get_new_point_in_ellipsoid(double goal_bias,
                                                       double c_best) {
    std::random_device rd;   // obtain a random number from hardware
    std::mt19937 gen(rd());  // seed the generator
    std::uniform_int_distribution<> x(0, 100);  // define the range
    double point_x = x(gen);
    point_x /= 100;
    if (point_x < goal_bias) {

      return std::pair<double, double>{goal->row, goal->col};
    }

    else {

      int c_min = dis(start, goal);
      int x1 = (start->row + goal->row) / 2;
      int x2 = (start->col + goal->col) / 2;
      MatrixXd x_center(2, 1);
      x_center(0) = x1;
      x_center(1) = x2;
      double node1_row_meter, node1_col_meter, node2_row_meter, node2_col_meter;
      global_costmap->mapToWorld(start->row, start->col, node1_row_meter,
                                 node1_col_meter);
      global_costmap->mapToWorld(goal->row, goal->col, node2_row_meter,
                                 node2_col_meter);
      double col = node2_col_meter - node1_col_meter;
      double row = node2_row_meter - node1_row_meter;
      double theta = atan2(col, row);
      MatrixXd C(2, 2);
      C(0, 0) = cos(theta);
      C(0, 1) = -sin(theta);
      C(1, 0) = sin(theta);
      C(1, 1) = cos(theta);
      double elem1 = c_best / 2;
      double elem2 = sqrt(pow(c_best, 2) - pow(c_min, 2));
      MatrixXd L(2, 2);
      L(0, 0) = elem1;
      L(0, 1) = 0;
      L(1, 0) = 0;

      L(1, 1) = elem2;

      double r = sqrt(point_x * 100);
      // Check r
      theta = 2 * M_PI * r;
      double x = r * cos(theta);
      double y = r * sin(theta);
      MatrixXd x_ball(2, 1);
      x_ball(0) = x;
      x_ball(1) = y;
      MatrixXd var(2, 2);
      var = C * L;
      MatrixXd x_rand = var * x_ball;
      x_rand = x_rand + x_center;
      std::pair<double, double> point{x_rand(0), x_rand(1)};
      return point;
    }
  }

  RRT_Node* get_nearest_node(std::pair<double, double> point) {
    // Get nearest point in the vertices to the current point based on row & col
    // values
    double dist{0}, dist_nearest{0};

    RRT_Node* nearest_node = new RRT_Node();
    RRT_Node* point_node = new RRT_Node();
    point_node->row = point.first;
    point_node->col = point.second;
    dist_nearest = 10000;
    int nearest_node_index{0};
    int count{0};
    for (const auto& node : vertices) {
      dist = dis(point_node, node);
      if (dist < dist_nearest) {
        dist_nearest = dist;
        nearest_node = node;
        nearest_node_index = count;
      }
    }
    count++;
    return nearest_node;
  }

  std::pair<double, double> sample(double goal_bias, double c_best) {
    std::pair<double, double> new_point;
    if (c_best <= 0) {
      new_point = get_new_point(goal_bias);

    }

    else {
      new_point = get_new_point_in_ellipsoid(goal_bias, c_best);
    }

    return new_point;
  }

  RRT_Node* extend(std::pair<double, double> new_point, double extend_dis) {
    RRT_Node* nearest_node = get_nearest_node(new_point);

    double nearest_row_meter, nearest_col_meter, new_row_meter, new_col_meter;
    global_costmap->mapToWorld(nearest_node->row, nearest_node->col,
                               nearest_row_meter, nearest_col_meter);
    global_costmap->mapToWorld(new_point.first, new_point.second, new_row_meter,
                               new_col_meter);
    double slope = atan2(new_col_meter - nearest_col_meter,
                         new_row_meter - nearest_row_meter);
    // ROS_INFO("Slope %lf", slope);

    double new_row = nearest_row_meter + extend_dis * cos(slope);
    double new_col = nearest_col_meter + extend_dis * sin(slope);
    unsigned int new_row_index, new_col_index;

    global_costmap->worldToMap(new_row, new_col, new_row_index, new_col_index);
    RRT_Node* new_node = new RRT_Node();
    new_node->row = new_row_index;
    new_node->col = new_col_index;
    new_row = new_row_index;
    new_col = new_col_index;
    if ((new_row >= 0 && new_row < size_row) &&
        (new_col >= 0 && new_col < size_col) &&
        (!check_collision(nearest_node, new_node))) {
      new_node->parent = nearest_node;
      new_node->cost = extend_dis;
      vertices.push_back(new_node);

      if (!found) {
        double d = dis(new_node, goal);

        if (d < min_dis){
          min_dis = d;
        }
                  ROS_INFO("Min distance is %lf", min_dis);

        if (d < 0.2) {
          goal->cost = d;
          goal->parent = new_node;
          vertices.push_back(goal);
          found = true;
        }
      }
      return new_node;
    }

    else
      //      ROS_INFO("Null");
      return nullptr;
  }
  // check
  std::vector<RRT_Node*> get_neighbors(RRT_Node* new_node, int neighbor_size) {
    std::pair<double, double> sample;
    std::vector<std::pair<double, double>> samples;
    std::vector<RRT_Node*> neighbors;
    for (const auto& itr : vertices) {
      // ITERATte over vertices and check if less than neighbor size
      if ((new_node->row != itr->row) || (new_node->col != itr->col)){
        if (dis(new_node, itr) < neighbor_size) {
        neighbors.push_back(itr);
      }
      }
    }
    return neighbors;
    // Remove the new node
  }

  double path_cost(RRT_Node* start_node, RRT_Node* end_node) {
    double cost{0};
    RRT_Node* curr_node = end_node;

    while ((start_node->row != curr_node->row) ||
           (start_node->col != curr_node->col)) {
      RRT_Node* parent = curr_node->parent;
      // Not sure about parent is none
      if (parent == nullptr) {
                ROS_ERROR("Invalid path %lf %lf %lf %lf", curr_node->row, curr_node->col, start_node->row, start_node->col);
        return 0;
      }

      cost += curr_node->cost;
      curr_node = parent;
    }

    return cost;
  }

 void rewire(RRT_Node* new_node, std::vector<RRT_Node*> neighbors) {
    if (neighbors.empty()) {
      return;
    }

    double distance = 0;
    std::vector<double> distances;
    std::vector<double> costs;
    for (const auto& itr : neighbors) {
      distance = dis(new_node, itr);
      distances.push_back(distance);
    //  ROS_WARN("First rewire %lf %lf", itr->row, itr->col);
      costs.push_back(distance + path_cost(start, itr));
    }
    std::vector<long unsigned int> indices = sort_indexes(costs);
    for (int i = 0; i < indices.size(); i++) {
      long unsigned int index = indices.at(i);
      if (!check_collision(new_node, neighbors[index])) {
        new_node->parent = neighbors[index];
        new_node->cost = distances[index];
        break;
      }
    }
    double new_cost;
    for (int i = 0; i < neighbors.size(); i++) {
      RRT_Node* node = neighbors.at(i);
     //       ROS_WARN("2 rewire");

      new_cost = path_cost(start, new_node) + distances[i];
       //     ROS_WARN("3 rewire");
      if ((path_cost(start, node) > new_cost) &&
          (!check_collision(node, new_node))) {

        node->parent = new_node;
        node->cost = distances[i];
      }
    }
  }
  std::vector<geometry_msgs::PoseStamped> getPath(RRT_Node* start_node, RRT_Node* end_node) {
    std::vector<geometry_msgs::PoseStamped> path;
    RRT_Node* curr_node = end_node;
    while ((start_node->row != curr_node->row) || (start_node->col != curr_node->col)){
      RRT_Node* parent = curr_node->parent;
      if (parent == nullptr){
        ROS_ERROR("Invalid path!");
        std::vector<geometry_msgs::PoseStamped> path_;
        return path_;
      }
      double curr_row_meter, curr_col_meter;
      global_costmap->mapToWorld(curr_node->row, curr_node->col, curr_row_meter, curr_col_meter);
      geometry_msgs::PoseStamped point;
      point.pose.position.x = curr_row_meter;
      point.pose.position.y = curr_col_meter;
      path.push_back(point);
      curr_node = parent;
    }
    std::reverse(path.begin(), path.end());
    return path;
  }

  std::vector<geometry_msgs::PoseStamped> informed_RRT_star(int n_pts, int neigbor_size) {
    double c_best{0};

    for (int i = 0; i < n_pts; i++) {
      if (found) {
        ROS_WARN("Path found");
        c_best = path_cost(start, goal);
      }
      std::pair<double, double> new_point = sample(0.2, c_best);
      RRT_Node* new_node = extend(new_point, 0.2);
      // Is not none
      if (new_node != nullptr) {

        std::vector<RRT_Node*> neighbors =
            get_neighbors(new_node, neigbor_size);
        rewire(new_node, neighbors);
      }
      ROS_INFO("%d", i);
    }
    if (found) {
      int steps = vertices.size() - 2;
      ROS_INFO("Path found, steps are %d", steps);
      double length = path_cost(start, goal);
      return getPath(start, goal);
    }

    else {
      ROS_ERROR("No path found %d", vertices.size());
      return {};
    }
  }
};
