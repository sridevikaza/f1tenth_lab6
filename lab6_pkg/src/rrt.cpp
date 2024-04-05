// This file contains the class definition of tree nodes and RRT
// Before you start, please read: https://arxiv.org/pdf/1105.1186.pdf
// Make sure you have read through the header file as well

#include "rrt/rrt.h"
using namespace std;

// Destructor of the RRT class
RRT::~RRT() {
    // Do something in here, free up used memory, print message, etc.
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "RRT shutting down");
}

// Constructor of the RRT class
RRT::RRT(): rclcpp::Node("rrt_node"), gen((std::random_device())()), tf_buffer(std::make_shared<tf2_ros::Buffer>(this->get_clock())), tf_listener(*tf_buffer) 
{
    // declare params
    this->declare_parameter<bool>("rrt_star", true);
    this->get_parameter("rrt_star", rrt_star);

    // ROS publishers
    drive_pub_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>("/drive", 10);
    og_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/grid", 10);
    waypoints_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/waypoints", 10);
    tree_pub = this->create_publisher<visualization_msgs::msg::MarkerArray>("/tree", 10);
    goal_pub = this->create_publisher<visualization_msgs::msg::Marker>("/goal", 10);
    steer_pub = this->create_publisher<visualization_msgs::msg::Marker>("/steer", 10);
    samples_pub = this->create_publisher<visualization_msgs::msg::Marker>("/samples", 10);
    path_pub = this->create_publisher<nav_msgs::msg::Path>("/path", 10);

    // ROS subscribers
    string pose_topic = "ego_racecar/odom";    
    string scan_topic = "/scan";
    if (rrt_star){
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(pose_topic, 1, std::bind(&RRT::pose_callback_rrt_star, this, std::placeholders::_1));
    }
    else{
        pose_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(pose_topic, 1, std::bind(&RRT::pose_callback_rrt, this, std::placeholders::_1));
    }
    scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
      scan_topic, 1, std::bind(&RRT::scan_callback, this, std::placeholders::_1));

    // timer to publish markers
    viz_timer = this->create_wall_timer(100ms, std::bind(&RRT::publish_markers, this));

    // occupancy grid
    og_ = nav_msgs::msg::OccupancyGrid();

    // car pose
    car_pose_ = geometry_msgs::msg::Pose();

    // create markers
    waypoints_markers = visualization_msgs::msg::MarkerArray();
    goal_marker = visualization_msgs::msg::Marker();

    // set variables
    grid_buffer = 15; // amount to dialte obstacles (cells)
    dist_size = 3.5; // ditribution size for random sampling (m)
    goal_thresh = 0.15; // distance for reaching goal (m)
    expansion_dist = 0.3; // steer expansion distance (m)
    num_points = 100; // num points for interpolate in collision check
    resolution = 0.01; // meters per cell in occupancy grid (m/cell)
    grid_size = 350; // height and width in occupancy grid (cells)
    num_samples = 1000; // number of sampled points (m)
    L_goal = 1.5; // lookahead dist for getting a goal waypoint (m)
    L_follow = 0.4; // lookahead dist for pure pursuit along RRT path (m)
    velocity = 0.5; // car velocity (m/s)
    min_steer = -M_PI/3; // clip steer for pure pursuit
    max_steer = M_PI/3; // clip steer for pure pursuit
    steering_gain = 0.3; // P gain for pure pursuit
    neighbor_thresh = 0.6; // RRT* threshold

    // load waypoints
    load_waypoints(x_points, y_points);

    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Created new RRT Object.");
}

// timer callback to keep publishing markers
void RRT::publish_markers() {
    waypoints_pub->publish(waypoints_markers);
    goal_pub->publish(goal_marker);
    og_pub_->publish(og_);
}

void RRT::scan_callback(const sensor_msgs::msg::LaserScan::ConstSharedPtr scan_msg) {
    // The scan callback, update your occupancy grid here
    // Args:
    //    scan_msg (*LaserScan): pointer to the incoming scan message
    // Returns:
    //

    // update your occupancy grid
    og_.header.frame_id = "ego_racecar/base_link";
    og_.info.map_load_time = this->now();
    og_.info.resolution = resolution; // m/cell
    og_.info.width = grid_size; // cells
    og_.info.height = grid_size;
    og_.info.origin.position.x = 0;
    og_.info.origin.position.y = -grid_size/2*resolution;
    og_.info.origin.position.z = 0.1;
    og_.data.assign(grid_size * grid_size, 0);
    auto region_size = resolution * grid_size;

    auto ranges = scan_msg->ranges.data();
    int ranges_size = scan_msg->ranges.size();
    
    for (int i=0; i<ranges_size; i++){

        auto angle = scan_msg->angle_min + i*scan_msg->angle_increment;
        auto x = ranges[i] * cos(angle);
        auto y = ranges[i] * sin(angle);

        if ((x<region_size) && (x>0) && (abs(y)<(region_size)/2)){

            int x_cell = int(x/resolution);
            int y_cell = int(y/resolution + grid_size/2);

            // dilate and set index
            for (int i=-grid_buffer; i<grid_buffer; i++){
                for (int j=-grid_buffer; j<grid_buffer; j++){
                    int x_idx = x_cell+i;
                    int y_idx = y_cell+j;
                    int idx = x_idx + y_idx*grid_size;
                    if (x_idx<grid_size && x_idx>0 && y_idx<grid_size && y_idx>0 && idx<pow(grid_size,2))
                    {
                        og_.data[idx] = 100;
                    }
                }
            }
        }
    }
}

void RRT::pose_callback_rrt(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    // The pose callback when subscribed to particle filter's inferred pose
    // The RRT main loop happens here
    // Args:
    //    pose_msg (*PoseStamped): pointer to the incoming pose message
    // Returns:

    car_pose_ = pose_msg->pose.pose;
    vector<RRT_Node> tree;

    // make start node
    RRT_Node start;
    start.x = 0;
    start.y = 0;
    start.is_root = true;
    tree.push_back(start);

    // get goal point
    auto goal = get_goal(car_pose_.position.x, car_pose_.position.y, x_points, y_points, L_goal);
    publish_goal(goal);

    // RRT initializations
    bool goal_found = false;
    int count = 0;
    RRT_Node new_node;
    clear_tree();

    while(!goal_found && count<num_samples){
        count++;

        // sample point
        auto sampled_point = sample();

        // get nearest node
        int tree_size = tree.size();
        int nearest_idx = nearest(tree, sampled_point);
        RRT_Node nearest_node;
        if (nearest_idx < tree_size && nearest_idx >= 0){
            nearest_node = tree[nearest_idx];
        }
        else{
            RCLCPP_INFO(this->get_logger(), "NEAREST NODE INDEX INVALID");
            continue;
        }
        
        // steer
        new_node = steer(nearest_node, sampled_point, tree);

        // check for collisions
        if (!check_collision(nearest_node, new_node))
        {
            tree.push_back(new_node);

            // check if goal is found
            if (is_goal(new_node, goal[0], goal[1])){
                goal_found = true;
            }
        }
    }

    // get path
    auto nodes = find_path(tree, new_node);
    publish_tree(tree);
    publish_path(nodes);

    // transform path
    vector<float> path_x_points, path_y_points;
    transform_path(path_x_points, path_y_points, nodes);

    // pure pursuit to follow path
    pure_pursuit(path_x_points, path_y_points, car_pose_);

}

void RRT::pose_callback_rrt_star(const nav_msgs::msg::Odometry::ConstSharedPtr pose_msg) {
    RCLCPP_INFO(this->get_logger(), "running rrt*");

    car_pose_ = pose_msg->pose.pose;
    vector<RRT_Node> tree;

    // make start node
    RRT_Node start;
    start.x = 0;
    start.y = 0;
    start.is_root = true;
    tree.push_back(start);

    // get goal point
    auto goal = get_goal(car_pose_.position.x, car_pose_.position.y, x_points, y_points, L_goal);
    publish_goal(goal);

    // RRT initializations
    bool goal_found = false;
    int count = 0;
    RRT_Node new_node;
    clear_tree();

    // rrt* loop
    while(!goal_found && count<num_samples){
        count++;

        // sample point
        auto sampled_point = sample();

        // get nearest node
        int tree_size = tree.size();
        int nearest_idx = nearest(tree, sampled_point);
        RRT_Node nearest_node;
        if (nearest_idx < tree_size && nearest_idx >= 0){
            nearest_node = tree[nearest_idx];
        }
        else{
            RCLCPP_INFO(this->get_logger(), "NEAREST NODE INDEX INVALID");
            continue;
        }
        
        // steer
        new_node = steer(nearest_node, sampled_point, tree);

        // get neighborhood
        auto neighborhood = near(tree, new_node);

        // find lowest cost neighbor
        double min_cost = numeric_limits<float>::max();
        int min_neighbor;
        for (auto idx : neighborhood){
            auto neighbor = tree[idx];
            double total_cost = cost(tree, neighbor) + line_cost(neighbor, new_node);
            if (total_cost < min_cost){
                min_cost = total_cost;
                min_neighbor = idx;
            }
        }

        // add to tree
        if (!check_collision(tree[min_neighbor], new_node)){
            new_node.parent = min_neighbor;
            tree.push_back(new_node);

            // check if goal is found
            if (is_goal(new_node, goal[0], goal[1])){
                goal_found = true;
                break;
            }

            // rewire
            for (auto idx : neighborhood){
                auto neighbor = tree[idx];
                if ((min_cost + line_cost(neighbor, new_node)) < cost(tree, neighbor) && !check_collision(neighbor, new_node)){
                    neighbor.parent = get_node_index(tree, new_node);
                }
            }
        }
    }

    // get path
    auto nodes = find_path(tree, new_node);
    publish_tree(tree);
    publish_path(nodes);

    // transform path
    vector<float> path_x_points, path_y_points;
    transform_path(path_x_points, path_y_points, nodes);

    // pure pursuit to follow path
    pure_pursuit(path_x_points, path_y_points, car_pose_);
}

void RRT::clear_tree() {
    visualization_msgs::msg::MarkerArray tree_markers;
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "ego_racecar/base_link";
    marker.header.stamp = this->get_clock()->now();  
    marker.action = 3; //visualization_msgs::msg::Marker::DELETEALL;
    marker.ns = "tree";
    tree_markers.markers.push_back(marker);

    tree_pub->publish(tree_markers);
}

void RRT::publish_tree(const std::vector<RRT_Node>& nodes) {

    visualization_msgs::msg::MarkerArray tree_markers;
    for (size_t i = 0; i < nodes.size(); ++i) {
        const auto& node = nodes[i];
        if (!node.is_root) {

            auto parent_node = nodes[node.parent];

            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "ego_racecar/base_link";
            marker.id = i;
            marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.ns = "tree";
            marker.scale.x = 0.01;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 0.0;
            marker.color.b = 1.0;

            // parent node
            geometry_msgs::msg::Point start;
            start.x = parent_node.x;
            start.y = parent_node.y;
            start.z = 0.0;

            // current node
            geometry_msgs::msg::Point end;
            end.x = node.x;
            end.y = node.y;
            end.z = 0.0;

            marker.points.push_back(start);
            marker.points.push_back(end);

            tree_markers.markers.push_back(marker);
        }
    }

    tree_pub->publish(tree_markers);
}

void RRT::publish_goal(vector<double> goal){
    // make goal marker
    goal_marker.header.frame_id = "ego_racecar/base_link";
    goal_marker.header.stamp = this->get_clock()->now();
    goal_marker.ns = "goal";
    goal_marker.type = visualization_msgs::msg::Marker::CYLINDER;
    goal_marker.action = visualization_msgs::msg::Marker::ADD;
    goal_marker.pose.position.x = goal[0];
    goal_marker.pose.position.y = goal[1];
    goal_marker.pose.position.z = 0.2;
    goal_marker.scale.x = 0.1;
    goal_marker.scale.y = 0.1;
    goal_marker.scale.z = 0.1;
    goal_marker.color.a = 1.0;
    goal_marker.color.r = 0.0;
    goal_marker.color.g = 1.0;
    goal_marker.color.b = 0.0;
}

std::vector<double> RRT::sample() {
    // This method returns a sampled point from the free space
    // You should restrict so that it only samples a small region
    // of interest around the car's current position
    // Args:
    // Returns:
    //     sampled_point (std::vector<double>): the sampled point in free space

    std::vector<double> sampled_point;

    x_dist.param(std::uniform_real_distribution<double>::param_type(0, dist_size));
    y_dist.param(std::uniform_real_distribution<double>::param_type(-dist_size/2, dist_size/2));

    sampled_point.push_back(x_dist(gen));
    sampled_point.push_back(y_dist(gen));

    return sampled_point;
}

double RRT::get_dist(double &x1, double &y1, double &x2, double &y2){
    return sqrt(pow(x2-x1,2) + pow(y2-y1,2));
}

int RRT::get_node_index(const std::vector<RRT_Node> &tree, const RRT_Node &node)
{
    int n = tree.size();
    const double EPSILON = 1e-6;
    for (int i=0; i<n; i++){
        if (std::abs(tree[i].x - node.x) < EPSILON && std::abs(tree[i].y - node.y) < EPSILON) {
            return i;
        }
    }
    RCLCPP_INFO(this->get_logger(), "NODE NOT FOUND IN GET_NODE_INDEX");
    return 0;
}

void RRT::transform_path(vector<float>& path_x_points, vector<float>& path_y_points, const vector<RRT_Node>& path){
    for (const auto node : path){

        // transform car frame to map frame
        geometry_msgs::msg::PointStamped map_point, car_point;
        car_point.header.frame_id = "ego_racecar/base_link";
        car_point.header.stamp = rclcpp::Time(0);
        car_point.point.x = node.x;
        car_point.point.y = node.y;
        car_point.point.z = 0.0;

        try {
            auto transformStamped = tf_buffer->lookupTransform("map", car_point.header.frame_id, car_point.header.stamp, rclcpp::Duration(1, 0));
            tf2::doTransform(car_point, map_point, transformStamped);
        }
        catch (tf2::TransformException &ex) {
            RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Loaded Waypoints");

            RCLCPP_ERROR(rclcpp::get_logger("RRT"), "Could not transform point: %s", ex.what());
        }

        path_x_points.push_back(map_point.point.x);
        path_y_points.push_back(map_point.point.y);
    }
}

void RRT::pure_pursuit(const vector<float>& path_x_points, const vector<float>& path_y_points, const geometry_msgs::msg::Pose& car_pose){

    // get next goal point in path
    auto goal = get_goal(car_pose.position.x, car_pose.position.y, path_x_points, path_y_points, L_follow);

    // calculate curvature/steering angle
    float gamma = 2 * goal[1] / pow(L_follow,2);
    float steering_angle = steering_gain * gamma;
    steering_angle = max(min_steer, min(steering_angle, max_steer)); // clip

    // publish drive message
    auto drive_msg = ackermann_msgs::msg::AckermannDriveStamped();
    drive_msg.drive.speed = velocity;
    drive_msg.drive.steering_angle = steering_angle;
    drive_pub_->publish(drive_msg);
}

int RRT::nearest(std::vector<RRT_Node> &tree, std::vector<double> &sampled_point) {
    // This method returns the nearest node on the tree to the sampled point
    // Args:
    //     tree (std::vector<RRT_Node>): the current RRT tree
    //     sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //     nearest_node (int): index of nearest node on the tree

    auto closest_dist = numeric_limits<float>::max();
    int nearest_node = 0;
    int n = tree.size();
    for (int i=0; i<n; i++){
        auto dist = get_dist(tree[i].x, tree[i].y, sampled_point[0], sampled_point[1]);
        if (dist < closest_dist){
            nearest_node = i;
            closest_dist = dist;
        }
    }

    return nearest_node;
}

RRT_Node RRT::steer(RRT_Node &nearest_node, std::vector<double> &sampled_point, std::vector<RRT_Node> &tree) {
    // The function steer:(x,y)->z returns a point such that z is “closer” 
    // to y than x is. The point z returned by the function steer will be 
    // such that z minimizes ||z−y|| while at the same time maintaining 
    //||z−x|| <= max_expansion_dist, for a prespecified max_expansion_dist > 0

    // basically, expand the tree towards the sample point (within a max dist)

    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    sampled_point (std::vector<double>): the sampled point in free space
    // Returns:
    //    new_node (RRT_Node): new node created from steering


    RRT_Node new_node = RRT_Node();

    if (get_dist(nearest_node.x, nearest_node.y, sampled_point[0], sampled_point[1]) < expansion_dist){
        new_node.x = sampled_point[0];
        new_node.y = sampled_point[1];
        new_node.parent = get_node_index(tree, nearest_node);
        return new_node;
    }

    auto dy = sampled_point[1] - nearest_node.y;
    auto dx = sampled_point[0] - nearest_node.x;
    auto angle = atan(dy/dx);

    new_node.x = expansion_dist * cos(angle) + nearest_node.x;
    new_node.y = expansion_dist * sin(angle) + nearest_node.y;
    new_node.parent = get_node_index(tree, nearest_node);

    return new_node;
}

bool RRT::check_collision(RRT_Node &nearest_node, RRT_Node &new_node) {
    // This method returns a boolean indicating if the path between the 
    // nearest node and the new node created from steering is collision free
    // Args:
    //    nearest_node (RRT_Node): nearest node on the tree to the sampled point
    //    new_node (RRT_Node): new node created from steering
    // Returns:
    //    collision (bool): true if in collision, false otherwise

    auto x1 = nearest_node.x;
    auto y1 = nearest_node.y;
    auto x2 = new_node.x;
    auto y2 = new_node.y;
    auto dx = x2-x1;
    auto dy = y2-y1;

    // interpolate over line
    for (int i=0; i<num_points; i++){
        auto x = x1 + dx/num_points * i;
        auto y = y1 + dy/num_points * i;

        // convert m to cell index
        int x_cell = x/resolution;
        int y_cell = y/resolution + grid_size/2;

        // check occupancy grid
        int idx = x_cell + y_cell*grid_size;
        if (x_cell<grid_size && x_cell>0 && y_cell<grid_size && y_cell>0 && idx<pow(grid_size,2)){
            if (og_.data[idx]>1){
                return true;
            }
        }
    }

    return false;
}

bool RRT::is_goal(RRT_Node &latest_added_node, double goal_x, double goal_y) {
    // This method checks if the latest node added to the tree is close
    // enough (defined by goal_threshold) to the goal so we can terminate
    // the search and find a path
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree
    //   goal_x (double): x coordinate of the current goal
    //   goal_y (double): y coordinate of the current goal
    // Returns:
    //   close_enough (bool): true if node close enough to the goal

    bool close_enough = false;

    if (get_dist(latest_added_node.x, latest_added_node.y, goal_x, goal_y) < goal_thresh){
        close_enough = true;
    }

    return close_enough;
}

std::vector<RRT_Node> RRT::find_path(std::vector<RRT_Node> &tree, RRT_Node &latest_added_node) {
    // This method traverses the tree from the node that has been determined
    // as goal
    // Args:
    //   latest_added_node (RRT_Node): latest addition to the tree that has been
    //      determined to be close enough to the goal
    // Returns:
    //   path (std::vector<RRT_Node>): the vector that represents the order of
    //      of the nodes traversed as the found path

    // initializations
    std::vector<RRT_Node> found_path;
    RRT_Node parent_node;
    int parent_idx; 

    // add latest node
    found_path.push_back(latest_added_node);
    parent_idx = latest_added_node.parent;

    // get all parent nodes
    while(parent_idx != 0){
        parent_node = tree[parent_idx];
        found_path.push_back(parent_node);
        parent_idx = parent_node.parent;
    }
    found_path.push_back(tree[parent_idx]);

    // reverse path and return
    reverse(found_path.begin(), found_path.end());
    return found_path;
}

void RRT::publish_path(vector<RRT_Node> nodes){
    nav_msgs::msg::Path path;
    path.header.frame_id = "ego_racecar/base_link";
    path.header.stamp = this->now();

    for (const auto& node : nodes){
        geometry_msgs::msg::PoseStamped pose;
        pose.header.frame_id = "ego_racecar/base_link";
        pose.header.stamp = path.header.stamp;
        pose.pose.position.x = node.x;
        pose.pose.position.y = node.y;
        pose.pose.position.z = 0.1;
        pose.pose.orientation.w = 1.0;
        path.poses.push_back(pose);
    }

    path_pub->publish(path);
}


// load pre-recorded waypoints from csv
void RRT::load_waypoints(vector<float> &x_points, vector<float> &y_points)
{
    ifstream csv_file("/sim_ws/src/f1tenth_lab5/waypoints/waypoints2.csv");
    string line;
    size_t id = 0;
    int line_count = 0;
    int skip = 200;

    // read each csv line
    while (getline(csv_file, line))
    {
        ++line_count;
        if (line_count % skip != 0) continue;

        stringstream line_stream(line);
        string cell;
        vector<float> waypoint;

        while (getline(line_stream, cell, ','))
        {
            waypoint.push_back(stof(cell));
        }

        if (waypoint.size() >= 2) // x, y
        {
            // make waypoint markers
            visualization_msgs::msg::Marker marker;
            marker.header.frame_id = "map";
            marker.header.stamp = this->get_clock()->now();
            marker.ns = "waypoints";
            marker.id = id++;
            marker.type = visualization_msgs::msg::Marker::CYLINDER;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose.position.x = waypoint[0];
            marker.pose.position.y = waypoint[1];
            marker.pose.position.z = 0.2;
            marker.scale.x = 0.1; 
            marker.scale.y = 0.1; 
            marker.scale.z = 0.1;
            marker.color.a = 1.0;
            marker.color.r = 0.0;
            marker.color.g = 1.0;
            marker.color.b = 0.0;

            // save marker values
            waypoints_markers.markers.push_back(marker);
            x_points.push_back(waypoint[0]);
            y_points.push_back(waypoint[1]);
        }
    }
    // waypoints_pub->publish(waypoints_markers);
    RCLCPP_INFO(rclcpp::get_logger("RRT"), "%s\n", "Loaded Waypoints");
}

// get the next goal waypoint for RRT to plan to
vector<double> RRT::get_goal(const double& current_x, const double& current_y, const vector<float>& x_values, const vector<float>& y_values, const double lookahead)
{
    // find closest waypoint to current position
    int num_waypoints = x_values.size();
    int closest_idx = 0;
    float closest_dist = numeric_limits<float>::max();
    for (int i=0; i<num_waypoints; ++i)
    {
        float dist = sqrt(pow(x_values[i]-current_x, 2) + pow(y_values[i]-current_y, 2));
        if (dist < closest_dist)
        {
            closest_idx = i;
            closest_dist = dist;
        }
    }

    // iterate through waypoints until past the lookahead dist to find next goal point
    bool found_goal = false;
    int goal_idx = closest_idx;
    while(!found_goal && goal_idx < num_waypoints)
    {
        ++goal_idx;
        float goal_dist = sqrt(pow(x_values[goal_idx]-current_x, 2) + pow(y_values[goal_idx]-current_y, 2));
        if (goal_dist > lookahead)
        {
            found_goal = true;
        }
    }

    // transform goal to car world
    geometry_msgs::msg::PointStamped map_goal, car_goal;
    map_goal.header.frame_id = "map";
    map_goal.header.stamp = rclcpp::Time(0);
    map_goal.point.x = x_values[goal_idx];
    map_goal.point.y = y_values[goal_idx];
    map_goal.point.z = 0.0;

    try {
        auto transformStamped = tf_buffer->lookupTransform("ego_racecar/base_link", map_goal.header.frame_id, map_goal.header.stamp, rclcpp::Duration(1, 0));
        tf2::doTransform(map_goal, car_goal, transformStamped);
    }
    catch (tf2::TransformException &ex) {
        RCLCPP_ERROR(this->get_logger(), "Could not transform point: %s", ex.what());
    }

    vector<double> goal;
    goal.push_back(car_goal.point.x);
    goal.push_back(car_goal.point.y);

    return goal;
}


// RRT* methods
double RRT::cost(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the cost associated with a node
    // Args:
    //    tree (std::vector<RRT_Node>): the current tree
    //    node (RRT_Node): the node the cost is calculated for
    // Returns:
    //    cost (double): the cost value associated with the node

    double cost = 0;
    auto child = node;
    
    while (!child.is_root){
        auto parent = tree[child.parent];
        cost += get_dist(child.x, child.y, parent.x, parent.y);
        child = parent;
    }

    return cost;
}

double RRT::line_cost(RRT_Node &n1, RRT_Node &n2) {
    // This method returns the cost of the straight line path between two nodes
    // Args:
    //    n1 (RRT_Node): the RRT_Node at one end of the path
    //    n2 (RRT_Node): the RRT_Node at the other end of the path
    // Returns:
    //    cost (double): the cost value associated with the path

    return get_dist(n1.x, n1.y, n2.x, n2.y);
}

std::vector<int> RRT::near(std::vector<RRT_Node> &tree, RRT_Node &node) {
    // This method returns the set of Nodes in the neighborhood of a 
    // node.
    // Args:
    //   tree (std::vector<RRT_Node>): the current tree
    //   node (RRT_Node): the node to find the neighborhood for
    // Returns:
    //   neighborhood (std::vector<int>): the index of the nodes in the neighborhood

    std::vector<int> neighborhood;

    int n = tree.size();
    for (int i=0; i<n; i++){
        if (get_dist(tree[i].x, tree[i].y, node.x, node.y) < neighbor_thresh){
            neighborhood.push_back(i);
        }
    }

    return neighborhood;
}
