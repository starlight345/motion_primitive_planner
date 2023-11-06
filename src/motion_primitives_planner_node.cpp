#include "motion_primitives_planner/motion_primitives_planner_node.hpp"

/* ----- Class Functions ----- */
MotionPlanner::MotionPlanner(ros::NodeHandle& nh) : nh_(nh)
{
  // Subscriber
  subOccupancyGrid = nh.subscribe("/map/local_map/obstacle",1, &MotionPlanner::CallbackOccupancyGrid, this);
  // Publisher
  pubSelectedMotion = nh_.advertise<sensor_msgs::PointCloud2>("/points/selected_motion", 1, true);
  pubMotionPrimitives = nh_.advertise<sensor_msgs::PointCloud2>("/points/motion_primitives", 1, true);
  pubCommand = nh_.advertise<ackermann_msgs::AckermannDrive>("/car_1/command", 1, true);
  
};

MotionPlanner::~MotionPlanner() 
{    
    ROS_INFO("MotionPlanner destructor.");
}

/* ----- ROS Functions ----- */

void MotionPlanner::CallbackOccupancyGrid(const nav_msgs::OccupancyGrid& msg)
{
  // Subscribe to the map messages
  localMap = msg;
  // The origin of the map [m, m, rad].  This is the real-world pose of the cell (0,0) in the map.
  this->origin_x = msg.info.origin.position.x;
  this->origin_y = msg.info.origin.position.y;
  // Frame id of the map
  this->frame_id = msg.header.frame_id;
  // The map resolution [m/cell]
  this->mapResol = msg.info.resolution;
  // message flag
  bGetMap = true;
}

void MotionPlanner::PublishSelectedMotion(std::vector<Node> motionMinCost)
{
  // publish selected motion primitive as point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  for (auto motion : motionMinCost) {
    pcl::PointXYZI pointTmp;
    pointTmp.x = motion.x;
    pointTmp.y = motion.y;
    cloud_in_ptr->points.push_back(pointTmp);
  }

  sensor_msgs::PointCloud2 motionCloudMsg;
  pcl::toROSMsg(*cloud_in_ptr, motionCloudMsg);
  motionCloudMsg.header.frame_id = this->frame_id;
  motionCloudMsg.header.stamp = ros::Time::now();
  pubSelectedMotion.publish(motionCloudMsg);
}

void MotionPlanner::PublishMotionPrimitives(std::vector<std::vector<Node>> motionPrimitives)
{
  // publish motion primitives as point cloud
  pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_in_ptr(new pcl::PointCloud<pcl::PointXYZI>);

  for (auto& motionPrimitive : motionPrimitives) {
    for (auto motion : motionPrimitive) {
      pcl::PointXYZI pointTmp;
      pointTmp.x = motion.x;
      pointTmp.y = motion.y;
      cloud_in_ptr->points.push_back(pointTmp);
    }
  }
  
  sensor_msgs::PointCloud2 motionPrimitivesCloudMsg;
  pcl::toROSMsg(*cloud_in_ptr, motionPrimitivesCloudMsg);
  motionPrimitivesCloudMsg.header.frame_id = this->frame_id;
  motionPrimitivesCloudMsg.header.stamp = ros::Time::now();
  pubMotionPrimitives.publish(motionPrimitivesCloudMsg);
}

void MotionPlanner::PublishCommand(std::vector<Node> motionMinCost)
{
  /*
    TODO: Publish control commands
    - Two components in the AckermannDrive message is used: steering_angle and speed.
    - Compute steering angle using your controller or other method.
    - Calculate proper speed command
    - Publish data  
  */
  ackermann_msgs::AckermannDrive command;
  command.steering_angle = 0.0;
  command.speed = 0.0;
  pubCommand.publish(command);
  std::cout << "command steer/speed : " << command.steering_angle*180/M_PI << " " << command.speed << std::endl;
}

/* ----- Algorithm Functions ----- */

void MotionPlanner::Plan()
{
  // Motion generation
  std::vector<std::vector<Node>> motionPrimitives = GenerateMotionPrimitives(this->localMap);
  
  // Select motion
  std::vector<Node> motionMinCost = SelectMotion(motionPrimitives);

  // Publish data
  PublishData(motionMinCost, motionPrimitives);
}

std::vector<std::vector<Node>> MotionPlanner::GenerateMotionPrimitives(nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: Generate motion primitives
    - you can change the below process if you need.
    - you can calculate cost of each motion if you need.
  */

  // initialize motion primitives
  std::vector<std::vector<Node>> motionPrimitives;

  // number of candidates
  int num_candidates = this->MAX_DELTA*2 / this->DELTA_RESOL; // *2 for considering both left/right direction

  // max progress of each motion
  double maxProgress = this->MAX_PROGRESS;
  for (int i=0; i<num_candidates+1; i++) {
    // current steering delta
    double angle_delta = this->MAX_DELTA - i * this->DELTA_RESOL;

    // init start node
    Node startNode(0, 0, 0, angle_delta, 0, 0, 0, -1, false);
    
    // rollout to generate motion
    std::vector<Node> motionPrimitive = RolloutMotion(startNode, maxProgress, localMap);

    // add current motionPrimitive
    motionPrimitives.push_back(motionPrimitive);
  }

  return motionPrimitives;
}

std::vector<Node> MotionPlanner::RolloutMotion(Node startNode,
                                              double maxProgress,
                                              nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: rollout to generate a motion primitive based on the current steering angle
    - calculate cost terms here if you need
    - check collision / sensor range if you need
    1. Update motion node using current steering angle delta based on the vehicle kinematics equation.
    2. collision checking
    3. range checking
  */

  // Initialize motionPrimitive
  std::vector<Node> motionPrimitive;
  // Init current motion node
  Node currMotionNode(startNode.x, startNode.y, 0, startNode.delta, 0, 0, 0, -1, false);
  // Start from small progress value
  double progress = this->DIST_RESOL;
  // 1. Update motion node using current steering angle delta based on the vehicle kinematics equation
  // - while loop until maximum progress of a motion
  while (progress < maxProgress) {
    // - you can use params in header file (MOTION_VEL, WHEELBASE, TIME_RESOL)
    // - steering angle value is 'startNode.delta' or 'currMotionNode.delta'
    // x_t+1   := x_t + x_dot * dt
    // y_t+1   := y_t + y_dot * dt
    // yaw_t+1 := yaw_t + yaw_dot * dt
    currMotionNode.x += 0.0;
    currMotionNode.y += 0.0;
    currMotionNode.yaw += 0.0;

    // 2. collision checking
    // - local to map coordinate transform
    Node collisionPointNode(currMotionNode.x, currMotionNode.y, currMotionNode.yaw, currMotionNode.delta, 0, 0, 0, -1, false);
    Node collisionPointNodeMap = LocalToMapCorrdinate(collisionPointNode);
    if (CheckCollision(collisionPointNodeMap, localMap)) {
      // - do some process when collision occurs.
      // - you can save collision information & calculate collision cost here.
      // - you can break and return current motion primitive or keep generate rollout.
    }

    // 3. range checking
    // - if you want to filter out motion points out of the sensor range, calculate the line-of-sight (LOS) distance & yaw angle of the node
    // - LOS distance := sqrt(x^2 + y^2)
    // - LOS yaw := atan2(y, x)
    // - if LOS distance > MAX_SENSOR_RANGE or abs(LOS_yaw) > FOV*0.5 <-- outside of sensor range 
    // - if LOS distance <= MAX_SENSOR_RANGE and abs(LOS_yaw) <= FOV*0.5 <-- inside of sensor range
    // - use params in header file (MAX_SENSOR_RANGE, FOV)
    double LOS_DIST = 0.0;
    double LOS_YAW = 0.0;
    if (LOS_DIST > this->MAX_SENSOR_RANGE || abs(LOS_YAW) > this->FOV*0.5) {
      // -- do some process when out-of-range occurs.
      // -- you can break and return current motion primitive or keep generate rollout.
    } 

    // append collision-free motion in the current motionPrimitive
    motionPrimitive.push_back(currMotionNode);

    // update progress of motion
    progress += this->DIST_RESOL;
  }
  
  // return current motion
  return motionPrimitive;
}

std::vector<Node> MotionPlanner::SelectMotion(std::vector<std::vector<Node>> motionPrimitives)
{
  /*
  TODO: select the minimum cost motion primitive
  
    1. Calculate cost terms
    2. Calculate total cost (weighted sum of all cost terms)
    3. Compare & Find minimum cost (double minCost) & minimum cost motion (std::vector<Node> motionMinCost)
    4. Return minimum cost motion
  */
  // Initialization
  double minCost = 9999999;
  std::vector<Node> motionMinCost; // initialize as odom

  // check size of motion primitives
  if (motionPrimitives.size() != 0) {
    // Iterate all motion primitive (motionPrimitive) in motionPrimitives
    for (auto& motionPrimitive : motionPrimitives) {
      // 1. Calculate cost terms

      // 2. Calculate total cost
      double cost_total = 0.0;

      // 3. Compare & Find minimum cost & minimum cost motion
      if (cost_total < minCost) {
          motionMinCost = motionPrimitive;
          minCost = cost_total;
      }
    }
  }
  // 4. Return minimum cost motion
  return motionMinCost;
}


/* ----- Util Functions ----- */

bool MotionPlanner::CheckCollision(Node currentNodeMap, nav_msgs::OccupancyGrid localMap)
{
  /*
    TODO: check collision of the current node
    - the position x of the node should be in a range of [0, map width]
    - the position y of the node should be in a range of [0, map height]
    - check all map values within the inflation area of the current node
    e.g.,

    for loop i in range(0, inflation_size)
      for loop j in range(0, inflation_size)
        tmp_x := currentNodeMap.x + i - 0.5*inflation_size <- you need to check whether this tmp_x is in [0, map width]
        tmp_y := currentNodeMap.y + j - 0.5*inflation_size <- you need to check whether this tmp_x is in [0, map height]
        map_index := "index of the grid at the position (tmp_x, tmp_y)" <-- map_index should be int, not double!
        map_value = static_cast<int16_t>(localMap.data[map_index])
        if (map_value > map_value_threshold) OR (map_value < 0)
          return true
    return false

    - use params in header file: INFLATION_SIZE, OCCUPANCY_THRES
  */

  return false;
}

bool MotionPlanner::CheckRunCondition()
{
  if (this->bGetMap) {
    return true;
  }
  else {
    std::cout << "Run condition is not satisfied!!!" << std::endl;
    return false;
  }
}

Node MotionPlanner::LocalToMapCorrdinate(Node nodeLocal)
{
  /*
    TODO: Transform from local to occupancy grid map coordinate
    - local coordinate ([m]): x [map min x, map max x], y [map min y, map max y]
    - map coordinate ([cell]): x [0, map width], y [map height]
    - convert [m] to [cell] using map resolution ([m]/[cell])
  */
  // Copy data nodeLocal to nodeMap
  Node nodeMap;
  memcpy(&nodeMap, &nodeLocal, sizeof(struct Node));
  // Transform from local (min x, max x) [m] to map (0, map width) [grid] coordinate
  nodeMap.x = 0;
  // Transform from local (min y, max y) [m] to map (0, map height) [grid] coordinate
  nodeMap.y = 0;

  return nodeMap;
}


/* ----- Publisher ----- */

void MotionPlanner::PublishData(std::vector<Node> motionMinCost, std::vector<std::vector<Node>> motionPrimitives)
{
  // Publisher
  // - visualize selected motion primitive
  PublishSelectedMotion(motionMinCost);

  // - visualize motion primitives
  PublishMotionPrimitives(motionPrimitives);

  // - publish command
  PublishCommand(motionMinCost);
}

/* ----- Main ----- */

int main(int argc, char* argv[])
{ 
  std::cout << "start main process" << std::endl;

  ros::init(argc, argv, "motion_primitives_planner");
  // for subscribe
  ros::NodeHandle nh;
  ros::Rate rate(50.0);
  MotionPlanner MotionPlanner(nh);

  // Planning loop
  while (MotionPlanner.nh_.ok()) {
      // Spin ROS
      ros::spinOnce();
      // check run condition
      if (MotionPlanner.CheckRunCondition()) {
        // Run algorithm
        MotionPlanner.Plan();
      }
      rate.sleep();
  }

  return 0;

}
