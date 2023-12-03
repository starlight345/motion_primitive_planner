/*
    Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
    Following tool is licensed under the terms and conditions of the ISC license.
    For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __CONTROL_SPACE_NODE_HPP__
#define __CONTROL_SPACE_NODE_HPP__

#include <iostream>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <string>
#include <fstream>

// pcl
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/pcd_io.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include "tf/transform_datatypes.h"
#include <tf2_msgs/TFMessage.h>
#include <tf/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
// ros
#include "ros/ros.h"
#include <nav_msgs/GetMap.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/PointCloud2.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int16.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8MultiArray.h>
#include <visualization_msgs/Marker.h>
#include <nav_msgs/OccupancyGrid.h>
#include <ackermann_msgs/AckermannDrive.h>

// Utils
double normalizePiToPi(float angle)
{
  return std::fmod(angle + M_PI, 2 * M_PI) - M_PI; // (angle + pi) % (2 * pi) - pi;
}

class Node
{
  public:
    // The default constructor for 3D array initialization
    Node(): Node(0, 0, 0, 0, 0, 0, 0, -1, false) {}
    // Constructor for a node with the given arguments
    Node(double x, double y, double yaw, double delta,
          double cost_dir, double cost_colli, double cost_total,
          int idx, bool collision) {
        this->x = x;
        this->y = y;
        this->yaw = yaw;
        this->delta = delta;
        this->cost_dir = cost_dir;
        this->cost_colli = cost_colli;
        this->cost_total = cost_total;
        this->idx = idx;
        this->collision = collision;
        this->traverse_cost = 0;
    }

    // the x position
    double x;
    // the y position
    double y;
    // the heading yaw
    double yaw;
    // the steering angle delta
    double delta;
    // the direction cost
    double cost_dir;
    // the collision cost
    double cost_colli;
    // the total cost
    double cost_total;
    // traverse cost
    double traverse_cost;
    // the index on path
    int idx;
    // flag for collision
    bool collision;
};

class MotionPlanner
{
  public:
    MotionPlanner(ros::NodeHandle& nh);        
    ~MotionPlanner();
    // ROS node
    ros::NodeHandle nh_;
    // Callback
    void CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg);
    // Publihsher
    void PublishSelectedMotion(std::vector<Node> motionMinCost);
    void PublishMotionPrimitives(std::vector<std::vector<Node>> motionPrimitives);
    void PublishCommand(std::vector<Node> motionMinCost);
    // Algorithms
    void Plan();
    std::vector<std::vector<Node>> GenerateMotionPrimitives(nav_msgs::OccupancyGrid localMap);
    std::vector<Node> RolloutMotion(Node startNode, double maxProgress, nav_msgs::OccupancyGrid localMap);
    std::vector<Node> SelectMotion(std::vector<std::vector<Node>> motionPrimitives);

    // Utils
    bool CheckCollision(Node currentNodeMap, nav_msgs::OccupancyGrid localMap, double inflation_size);
    Node LocalToMapCorrdinate(Node nodeLocal);
    bool CheckRunCondition();
    void PublishData(std::vector<Node> motionMinCost, std::vector<std::vector<Node>> motionPrimitives);
    
    // Parameters
    double FOV = 85.2 * (M_PI / 180.0); // [rad] FOV of point cloud (realsense)
    double MAX_SENSOR_RANGE = 10.0; // [m] maximum sensor range (realsense)
    double WHEELBASE = 0.27; // [m] wheelbase of the vehicle (our RC vehicle is near 0.26 m)
    // TODO: Change (tune) below parameters if you need
    double DIST_RESOL = 0.2; // [m] distance resolution for rollout
    double TIME_RESOL = 0.05; // [sec] time resolution between each motion (for rollout)
    double MOTION_VEL = DIST_RESOL / TIME_RESOL; // [m/s] velocity between each motion (for rollout)
    double DELTA_RESOL = 0.5 * (M_PI / 180.0); // [rad] angle resolution for steering angle sampling
    double MAX_DELTA = 25.0 * (M_PI / 180.0); // [rad] maximum angle for steering angle sampling
    double MAX_PROGRESS = 10.0; // [m] max progress of motion
    double INFLATION_SIZE = 0.6 / DIST_RESOL; // [grid] inflation size [m] / grid_res [m/grid]

    // Map info
    // TODO: Match below parameters with your cost map 
    double mapMinX =  -5; // [m] minimum x position of the map
    double mapMaxX =  15; // [m] maximum x position of the map
    double mapMinY = -10; // [m] minimum y position of the map
    double mapMaxY =  10; // [m] maximum y position of the map
    double mapResol = 0.1; // [m/cell] The map resolution
    double origin_x = 0.0; // [m] x position of the map origin
    double origin_y = 0.0; // [m] y position of the map origin
    std::string frame_id = "base_link"; // frame id of the map
    int OCCUPANCY_THRES = 50; // occupancy value threshold


    double prev_delta = 0;
    double prev_delta_2 = 0;

    int count = 0;

    double prev_error_y = 0;
    double total_error_y = 0;
    
  private:
    // Input
    ros::Subscriber subOccupancyGrid;
    // Output
    ros::Publisher pubSelectedMotion;
    ros::Publisher pubMotionPrimitives;
    ros::Publisher pubCommand;
    
    // I/O Data
    nav_msgs::OccupancyGrid localMap;

    // Signal checker
    bool bGetMap = false;
};


#endif // __CONTROL_SPACE_NODE_HPP__