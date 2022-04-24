/*
Inputs:
Start position
Cost Map - ros costmap pointer

Outputs: 
path - std::vector<geometry_msgs>
*/
#include <vector>
#include <cmath>
#include <iostream>
#include <Eigen/Dense>
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <random>

using Eigen::MatrixXd;

void costmap_callback(const nav_msgs/OccupancyGrid::ConstPtr& msg)
{
    
}

struct Node
{
    public:
    double row;
    double col;
    Node* parent;
    double cost;
};

class RRT
{
    RRT(Node* start_, Node* goal_, auto costmap_2d::Costmap2D *global_costmap_)
    {
        ros::NodeHandle n;
        ros::Subscriber sub = n.subscribe("nav_msgs/global_costmap/costmap", 1, costmap_callback);   
        global_costmap = global_costmap_;
        size_row = gloabl_costmap->getSizeInCellsX();
        size_col = global_costmap->getSizeInCellsY();
        vertices.push_back(start_);
        start = start_;
        goal = goal_;
    }

    // Cost Map
    // Working with Cell indexes
    int size_row;
    int size_col;
    Node* start;
    Node* goal;
    std::vector<Node*> vertices;
    bool found{false};
    costmap_2d::Costmap2D *global_costmap;

    double dis(Node* node1, Node* node2)
    {
        return sqrt(pow((node1->row - node2->row), 2) + pow((node1->col - node2->col), 2));
    }

    bool CollisionDetector::isThisPointCollides(double wx, double wy) {
        // In case of no costmap loaded
        if (global_costmap == nullptr) {
            // no collision
            return false;
        }

        int mx{0}, my{0};
        worldToMap(wx, wy, mx, my);

        if ((mx < 0) || (my < 0) || (mx >= global_costmap->getSizeInCellsX()) || (my >= global_costmap->getSizeInCellsY()))
            return true;

        // getCost returns unsigned char
        unsigned int cost = static_cast<int>(global_costmap->getCost(mx, my));

        if (cost > 0)
            return true;

        return false;
    }
    //Node 2 is the new point
    bool check_collision(Node* node1, Node* node2)
    {
        // In case of no costmap loaded
        if (global_costmap == nullptr) {
            // there is NO obstacles
            return false;
        }

        double dist = dis(node1, node2);
        resolution_;

        if (dist < resolution_) {
            return (isThisPointCollides(node2->row, node2->col)) ? true : false;
        } else {
            int steps_number = static_cast<int>(floor(dist/resolution_));
            double theta = atan2(node1->col - node2->col, node1->row - node2->row);
            std::pair<double, double> p_n;
            for (int n = 1; n < steps_number; n++) {
            p_n.first = node1->row + n * resolution_ * cos(theta);
            p_n.second = node1->col + n * resolution_ * sin(theta);
            if (isThisPointCollides(p_n.first, p_n.second))
                return true;
            }
            return false;
        }
    }

    std::pair<double, double> get_new_point(double goal_bias)
    {
        //Generate a pair of random numbers TODO
        std::random_device rd; // obtain a random number from hardware
        std::mt19937 gen(rd()); // seed the generator
        std::uniform_int_distribution<> x0(0, 100); // define the range
        double point_ = x0(gen);
        point_ /= 100;
        
        if(point_ < goal_bias)
        {
            return std::pair<double, double> {goal->row, goal->col};
        }

        else
        {
            std::uniform_int_distribution<> x(0, size_row - 1), y(0, size_col - 1); // define the range
            double point_x = x(gen);
            double point_y = y(gen); // generate numbers
            return std::pair<double, double> {point_x, point_y};
        }

    }

    std::pair<double, double> get_new_point_in_ellipsoid(double goal_bias, double c_best)
    {
        std::random_device rd; // obtain a random number from hardware
        std::mt19937 gen(rd()); // seed the generator
        std::uniform_int_distribution<> x(0, 100); // define the range
        double point_x = x(gen);
        point_x /= 100;
        
        if(point_x < goal_bias)
        {
            return std::pair<double, double> {goal->row, goal->col};
        }

        else
        {
            int c_min = dis(start, goal);
            int x1 = (start->row + goal->row) / 2;
            int x2 = (start->col + goal->col) / 2;
            MatrixXd x_center(1, 2);
            x_center(0, 0) = x1;
            x_center(0, 1) = x2;
            int col = goal->col - start->col;
            int row = goal->row - start->row;
            double theta = atan2(col, row);
            MatrixXd C{{cos(theta), -sin(theta)}, {sin(theta), cos(theta)}};
            double elem1 = c_best / 2;
            double elem2 = sqrt(pow(c_best, 2) - pow(c_min, 2));
            MatrixXd L{{elem1, 0}, {0, elem2}};
            double r = sqrt(point_ * 100);
            // Check r
            theta = 2 * M_PI * r;
            double x = r * cos(theta);
            double y = r * sin(theta);
            MatrixXd x_ball{x, y};
            MatrixXd var{C * L};
            MatrixXd x_rand = var * x_ball + x_center;

            std::pair<double, double> point {x_rand(0, 0), x_rand(0, 1)};
            return point;
        }
    }


    Node* get_nearest_node(std::pair<double, double> point)
    {
        //Get nearest point in the vertices to the current point based on row & col values
        double dist{0}, dist_nearest{0};
        std::vector<Node*>::iterator itr;
        Node* nearest_node = new Node();
        Node* point_node = new Node();
        point_node->row = point.first;
        point_node->col = point.second;
        dist_nearest = dis(point_node, vertices[0]);
        int nearest_node_index{0};
        // What do we want to return?
        int count{0};
        for(itr = vertices.begin(); itr < vertices.end(); itr++)
        {
            dist = dis(point_node, *itr);
            if(dist < dist_nearest)
            {
                dist_nearest = dist;
                nearest_node = *itr;
                nearest_node_index = count;
            }

            count++;
        }

        return nearest_node;
    }

    std::pair<double, double> sample(double goal_bias, double c_best)
    {
        std::pair<double, double> new_point;
        if (c_best <= 0)
        {
            new_point = get_new_point(goal_bias);
        }

        else
        {
            new_point = get_new_point_in_ellipsoid(goal_bias, c_best);
        }

        return new_point;
    }

    Node* extend(std::pair<double, double> new_point, double extend_dis)
    {
        Node* nearest_node = get_nearest_node(new_point);
        double slope = atan2(new_point.second - nearest_node->col, new_point.first - nearest_node->row);
        double new_row = nearest_node->row + extend_dis * cos(slope);
        double new_col = nearest_node->col + extend_dis * sin(slope);
        Node* new_node = new Node();
        new_node->row = new_row;
        new_node->col = new_col;

        if((new_row >= 0 && new_row < size_row) && (new_col >= 0 && new_col < size_col) && (!check_collision(nearest_node, new_node)))
        {
            new_node->parent = nearest_node;
            new_node->cost = extend_dis;
            vertices.push_back(new_node);

            if(!found)
            {
                double d = dis(new_node, goal);
                if(d < extend_dis)
                {
                    goal->cost = d;
                    goal->parent = new_node;
                    vertices.push_back(goal);
                    found = true;
                }
            }
            return new_node;
        }

        else
            return 0;

    }

    std::vector<Node*> get_neighbors(Node* new_node, int neighbor_size)
    {
        std::pair<double, double> sample;
        std::vector<std::pair<double, double>> samples;
        std::vector<Node*>::iterator itr;
        double row{0}, col{0};
        for(itr = vertices.begin(); itr < vertices.end(); itr++)
        {
            row = (*itr)->row;
            col = (*itr)->col;
            sample = {row, col};
            samples.push_back(sample);
        }

        std::vector<std::pair<double, double>>::iterator vec_itr;
        std::vector<int> neighbors_indices;
        int count{0};
        for(vec_itr = samples.begin(); vec_itr < samples.end(); itr++)
        {
            if(sqrt(pow((*vec_itr).first - new_node->row, 2) + pow((*vec_itr).second - new_node->col, 2)) < neighbor_size)
            {
                neighbors_indices.push_back(count);
            }
            count++;
        }

        std::vector<Node*> neighbors;
        //Add nodes from vertices to neighbors at all indices in neighbors_indices
        std::vector<int>::iterator indices_itr;
        for(indices_itr = neighbors_indices.begin(); indices_itr < neighbors_indices.end(); indices_itr++)
        {
            neighbors.push_back(vertices[(*indices_itr)]);
        }

        return neighbors;        
        //Remove the new node

    }

    double path_cost(Node* start_node, Node* end_node)
    {
        double cost{0};
        Node* curr_node = end_node;

        while((start_node->row != curr_node->row) || (start_node->col != curr_node->col))
        {
            Node* parent = curr_node->parent;

            //Not sure about parent is none
            if(parent == nullptr)
            {
                return 0;
            }

            cost += curr_node->cost;
            curr_node = parent;
        }

        return cost;
    }

    void rewire(Node* new_node, std::vector<Node*> neighbors)
    {
        if(neighbors.empty())
        {
            return;
        }

        std::vector<double> distances;
        double distance{0};
        std::vector<Node*>::iterator itr;
        for(itr = vertices.begin(); itr < vertices.end(); itr++)
        {
            distance = dis(new_node, *itr);
            distances.push_back(distance);
        }
    }

    void informed_RRT_star(int n_pts, int neigbor_size)
    {
        init_map();
        double c_best{0};

        for(int i = 0; i < n_pts; i++)
        {
            if(found)
            {
                c_best = path_cost(start, goal);
            }

            std::pair<double, double> new_point = sample(0.05, c_best);
            Node* new_node = extend(new_point, 10);
            //Is not none
            if(new_node != nullptr)
            {
                std::vector<Node*> neighbors = get_neighbors(new_node, neigbor_size);
                rewire(new_node, neighbors);
            }
        }

        if(found)
        {
            int steps = vertices.size() - 2;
            double length = path_cost(start, goal);
            std::cout << "It took " << steps << "nodes to find the current path" << std::endl;
            std::cout << "The path length is " << length << std::endl;

        }

        else
        {
            std::cout << "No path found";
        }
        
    }
};
