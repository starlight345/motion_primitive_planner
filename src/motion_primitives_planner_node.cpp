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
  Node last_node = motionMinCost.back();
  double delta = last_node.delta;


  command.steering_angle = delta;


  // start of PID control 
  int primitive_size = motionMinCost.size();

  Node look_ahead_node = motionMinCost[int(primitive_size/2)];

  double error_y = look_ahead_node.y;

  double kp = pow(10,(-1));
  double ki = pow(10,(-3));
  double kd = pow(10,(-1));

  double steer = kp*error_y + ki * this->total_error_y + kd * (error_y-this->prev_error_y);

  this->total_error_y += error_y;
  this->prev_error_y = error_y;

  // Control limit
  steer = std::min(std::max(steer, -this->MAX_DELTA), this->MAX_DELTA);

  // command.steering_angle = steer;

  // speed control 
  // if (this->count < 200) {
  //   command.speed = 0.4 - 0.3 * (abs(delta/this->MAX_DELTA));
  //   this->count += 1;
  // }
  // else {
  //   if (abs(delta*180/M_PI) < 2 && abs(this->prev_delta_2*180/M_PI) < 2){
  //     std::cout << abs(delta*180/M_PI) << std::endl;
  //     command.speed = 1 - 0.4 * (abs(delta/this->MAX_DELTA));
  //   }
  //   else {
  //     command.speed = 0.5 - 0.3 * (abs(delta/this->MAX_DELTA));
  //   }
  // }


  // command.steering_angle = 0;
  // command.speed = 0;
  // command.speed = 0.5 - 0.4 * (abs(steer*2/this->MAX_DELTA));

  command.speed = 0.4 * exp(-3.5 *abs(delta));

  
  this->prev_delta_2 = delta;
  pubCommand.publish(command);
  std::cout << "command steer/ speed/ delta : " << command.steering_angle*180/M_PI << " " << command.speed << " " << steer*180/M_PI << std::endl;
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
    double yaw_dot = tan(currMotionNode.delta) * this->MOTION_VEL / this->WHEELBASE;
    currMotionNode.x += cos(currMotionNode.yaw) * this->DIST_RESOL;
    currMotionNode.y += sin(currMotionNode.yaw) * this->DIST_RESOL;
    currMotionNode.yaw = normalizePiToPi(currMotionNode.yaw + yaw_dot * this->TIME_RESOL);
    
    currMotionNode.cost_dir = abs(currMotionNode.delta);


    Node collisionPointNode(currMotionNode.x, currMotionNode.y, currMotionNode.yaw, currMotionNode.delta, 0, 0, 0, -1, false);
    Node collisionPointNodeMap = LocalToMapCorrdinate(collisionPointNode);

    if (CheckCollision(collisionPointNodeMap, localMap, 1.2/this->DIST_RESOL)) {
      currMotionNode.traverse_cost += 1/pow((currMotionNode.idx+2),2);
    }
    
    // 2. collision checking
    // - local to map coordinate transform
    if (CheckCollision(collisionPointNodeMap, localMap, this->INFLATION_SIZE)) {
      // - do some process when collision occurs.
      currMotionNode.collision = true;
      currMotionNode.cost_colli = 1;
      currMotionNode.idx += 1;
      motionPrimitive.push_back(currMotionNode);
      break;
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
    double LOS_DIST = sqrt(pow(currMotionNode.x, 2) + pow(currMotionNode.y, 2));
    double LOS_YAW = atan2(currMotionNode.y, currMotionNode.x);
    if (LOS_DIST > this->MAX_SENSOR_RANGE || abs(LOS_YAW) > this->FOV*0.5) {
      break;
    }

    currMotionNode.idx += 1;
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
      Node last_node = motionPrimitive.back();
      
      double dist = sqrt(pow(last_node.x, 2) + pow(last_node.y, 2));
      double cost_prog = (1/dist);
      double cost_prog_norm = (cost_prog - 1/this->MAX_PROGRESS) / (1/this->DIST_RESOL - 1/this->MAX_PROGRESS);
      // double max_size = this->MAX_PROGRESS / this->DIST_RESOL;
      // double cost_prog_norm = motionPrimitive.size() / max_size;
      double cost_colli_norm = (last_node.collision == true) ? 1 : 0;

      double cost_dir_norm = (last_node.cost_dir) / (this->MAX_DELTA);

      double cur_delta = last_node.delta; 
      double delta_change = abs(cur_delta - this->prev_delta) / (2*this->MAX_DELTA);
      this->prev_delta = cur_delta;

      double traverse_cost = 0;
      for (size_t i = 0; i<motionPrimitive.size(); i++){
        traverse_cost += motionPrimitive[i].traverse_cost;
      }
      double traverse_cost_norm = traverse_cost/1.6;
      
      // 2. Calculate total cost
      double cost_total =  cost_dir_norm * 5 + cost_prog_norm * 20 + 1000 * cost_colli_norm + 5 * delta_change + 4 * traverse_cost_norm;

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

bool MotionPlanner::CheckCollision(Node currentNodeMap, nav_msgs::OccupancyGrid localMap, double inflation_size)
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

int map_width = int(this->mapMaxX - this->mapMinX + 0.5f);
map_width = int(map_width / this->mapResol + 0.5f);

 int map_height = int(this->mapMaxY - this->mapMinY + 0.5f);
  map_height = int(map_height/ this->mapResol) ;

  for (int i = 0; i <= inflation_size; i++) {
    for (int j = 0; j <= inflation_size; j++) {
      int tmp_x = int(currentNodeMap.x + i - 0.5*inflation_size + 0.5f);
      int tmp_y = int(currentNodeMap.y + j - 0.5*inflation_size + 0.5f);

      if (tmp_x >= 0 && tmp_x <= map_width && tmp_y >= 0 && tmp_y <= map_height) {
        int map_index = tmp_x + tmp_y * map_width;
        int map_value = static_cast<int16_t>(localMap.data[map_index]);
        if (map_value > this->OCCUPANCY_THRES || map_value < 0) {
          // std::cout << "collision : " << map_value << std::endl;
          return true;
        }
      }
    }
  }
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
  nodeMap.x = (nodeLocal.x - this->mapMinX) / this->mapResol;
  // Transform from local (min y, max y) [m] to map (0, map height) [grid] coordinate
  nodeMap.y = (nodeLocal.y - this->mapMinY) / this->mapResol;

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
