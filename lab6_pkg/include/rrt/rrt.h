// RRT assignment

// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf

#include <iostream>
#include <string>
#include <algorithm>
#include <sstream>
#include <cmath>
#include <vector>
#include <random>
#include <fstream>

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ackermann_msgs/msg/ackermann_drive_stamped.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "geometry_msgs/msg/point_stamped.h"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "nav_msgs/msg/path.hpp"
#include <tf2_ros/transform_broadcaster.h>
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>

/// CHECK: include needed ROS msg type headers and libraries

using namespace std;

// Struct defining the RRT_Node object in the RRT tree.
// More fields could be added to thiis struct if more info needed.
// You can choose to use this or not
typedef struct RRT_Node {
    double x, y;
    double cost; // only used for RRT*
    int parent; // index of parent node in the tree vector
    bool is_root = false;
} RRT_Node;

class RRT : public rclcpp::Node {
public:
    RRT();
    virtual ~RRT();

private:

    // publishers and subscribers
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr pose_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr drive_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr og_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr waypoints_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr goal_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr steer_pub;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr samples_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub;

    // timer
    rclcpp::TimerBase::SharedPtr viz_timer;

    // random generator
    std::mt19937 gen;
    std::uniform_real_distribution<> x_dist;
    std::uniform_real_distribution<> y_dist;
    
    // initializations
    std::shared_ptr<tf2_ros::Buffer> tf_buffer;
    tf2_ros::TransformListener tf_listener;
    nav_msgs::msg::OccupancyGrid og_;
    geometry_msgs::msg::Pose car_pose_;
    visualization_msgs::msg::MarkerArray waypoints_markers;
    visualization_msgs::msg::Marker goal_marker;
    vector<float> x_points;
    vector<float> y_points;
    double dist_size;
    double resolution;
    double L;
    double expansion_dist;
    double goal_thresh;
    int grid_buffer;
    int num_points;
    int grid_size;
    int num_samples;

    // callbacks
    // where rrt actually happens
    void pose_callback(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg);
    // updates occupancy grid
    void scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg);

    // RRT methods
    std::vector<double> sample();
    int nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point);
    RRT_Node steer(RRT_Node &nearest_node, std::vector<double> &sampled_point, std::vector<RRT_Node> &tree);
    bool check_collision(RRT_Node &nearest_node, RRT_Node &new_node);
    bool is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y);
    std::vector<RRT_Node> find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node);
    void publish_path(std::vector<RRT_Node> path);
    double get_dist(double &x1, double &y1, double &x2, double &y2);
    int get_node_index(const std::vector<RRT_Node> &tree, const RRT_Node &node);  
    vector<double> getGoal(const double& current_x, const double& current_y);
    void load_waypoints(vector<float> &x_points, vector<float> &y_points);
    void publish_markers();

    // RRT* methods
    double cost(std::vector<RRT_Node> &tree, RRT_Node &node);
    double line_cost(RRT_Node &n1, RRT_Node &n2);
    std::vector<int> near(std::vector<RRT_Node> &tree, RRT_Node &node);

};

