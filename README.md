# Motion Primitives-based Planning Algorithm. (ROS, C++)
Autonomous driving via motion primitives-based planner.

## TODO

1. Utility functions

    a. Collision Checking

        bool MotionPlanner::CheckCollision(Node currentNodeMap, nav_msgs::OccupancyGrid localMap)
        /*
            TODO: check collision of the current node
            - the position x of the node should be in a range of [0, map width]
            - the position y of the node should be in a range of [0, map height]
            - check all map values within the inflation area of the current node

            e.g.,
            for loop i in range(0, inflation_size)
                for loop j in range(0, inflation_size)
                    tmp_x := current_x + i - 0.5*inflation_size <- you need to check whether this tmp_x is in [0, map width]
                    tmp_y := current_y + j - 0.5*inflation_size <- you need to check whether this tmp_x is in [0, map height]
                    map_index := "index of the grid at the position (tmp_x, tmp_y)"
                    map_value := static_cast<int16_t>(localMap.data[map_index])
                    if (map_value > map_value_threshold) OR (map_value < 0)
                        return true
            return false   
        */
        
    b. Local to Map Coordinate

        Node MotionPlanner::LocalToMapCorrdinate(Node nodeLocal)
        /*
            TODO: Transform from local to occupancy grid map coordinate
            - local coordinate ([m]): x [map min x, map max x], y [map min y, map max y]
            - map coordinate ([cell]): x [0, map width], y [map height]
            - convert [m] to [cell] using map resolution ([m]/[cell])
        */


2. Main algorithms

    a. Generate motion primitives

        std::vector<std::vector<Node>> MotionPlanner::GenerateMotionPrimitives(nav_msgs::OccupancyGrid localMap)
        /*
            TODO: Generate motion primitives
            - you can change the below process if you need.
            - you can calculate cost of each motion if you need.
        */
        
    b. Compute rollout of a motion primitive

        std::vector<Node> MotionPlanner::RolloutMotion(Node startNode,
                                                    double maxProgress,
                                                    nav_msgs::OccupancyGrid localMap)
        /*
            TODO: rollout to generate a motion primitive based on the current steering angle
            - calculate cost terms here if you need
            - check collision / sensor range if you need
            1. Update motion node using current steering angle delta based on the vehicle kinematics equation.
            2. collision checking
            3. range checking
        */
        
    c. Calculate cost of motion primitives & Select cost-minimum motion
        
        std::vector<Node> MotionPlanner::SelectMotion(std::vector<std::vector<Node>> motionPrimitives)
        /*
            TODO: select the minimum cost motion primitive
            1. Calculate cost terms
            2. Calculate total cost (weighted sum of all cost terms)
            3. Compare & Find minimum cost (double minCost) & minimum cost motion (std::vector<Node> motionMinCost)
            4. Return minimum cost motion
        */
    
    d. Publish control commands

        void MotionPlanner::PublishCommand(std::vector<Node> motionMinCost)
        /*
            TODO: Publish control commands
            - Two components in the AckermannDrive message is used: steering_angle and speed.
            - Compute steering angle using your controller or other method.
            - Calculate proper speed command
            - Publish data  
        */

## Parameters (in motion_primitives_planner_node.hpp)
- Change (tune) parameters if you need
    
    ```
    // Parameters
    double FOV = 85.2 * (M_PI / 180.0); // [rad] FOV of point cloud (realsense)
    double MAX_SENSOR_RANGE = 10.0; // [m] maximum sensor range (realsense)
    double WHEELBASE = 0.27; // [m] wheelbase of the vehicle (our RC vehicle is near 0.26 m)
    // TODO: Change (tune) below parameters if you need
    double DIST_RESOL = 0.2; // [m] distance resolution for rollout
    double TIME_RESOL = 0.05; // [sec] time resolution between each motion (for rollout)
    double MOTION_VEL = DIST_RESOL / TIME_RESOL; // [m/s] velocity between each motion (for rollout)
    double DELTA_RESOL = 0.5 * (M_PI / 180.0); // [rad] angle resolution for steering angle sampling
    double MAX_DELTA = 10.0 * (M_PI / 180.0); // [rad] maximum angle for steering angle sampling
    double MAX_PROGRESS = 5.0; // [m] max progress of motion
    double INFLATION_SIZE = 0.5 / DIST_RESOL; // [grid] inflation size [m] / grid_res [m/grid]

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
    ```
    
## Subscribed Topics

* /map/local_map/obstacle <-- local cost map

## Published Topics

* /points/selected_motion <-- selected motion primitive for visualization
* /points/motion_primitives <-- motion primitives for visualization
* /car_1/command <-- control command

