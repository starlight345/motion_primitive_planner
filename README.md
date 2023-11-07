# Motion Primitives-based Planner Node (ROS, C++)

## Intro
Autonomous driving via motion primitives-based planner.

## Running
After building this ROS package using 'catkin_make' in your catkin workspace, launch the motion primitives-based planner node:

`roslaunch motion_primitives_planner run.launch`

## TODO

1. Utility functions

    a. Collision Checking
    https://github.com/hynkis/motion_primitives_planner/blob/831f817e3ddc13ac049eec9d81ec0a81d3d735e3/src/motion_primitives_planner_node.cpp#L243-L266
        
    b. Local to Map Coordinate
    https://github.com/hynkis/motion_primitives_planner/blob/831f817e3ddc13ac049eec9d81ec0a81d3d735e3/src/motion_primitives_planner_node.cpp#L279-L296


2. Main algorithms

    a. Generate motion primitives
    https://github.com/hynkis/motion_primitives_planner/blob/831f817e3ddc13ac049eec9d81ec0a81d3d735e3/src/motion_primitives_planner_node.cpp#L107-L138
        
    b. Compute rollout of a motion primitive
    https://github.com/hynkis/motion_primitives_planner/blob/831f817e3ddc13ac049eec9d81ec0a81d3d735e3/src/motion_primitives_planner_node.cpp#L140-L204
        
    c. Calculate the cost of motion primitives and select cost-minimum motion
    https://github.com/hynkis/motion_primitives_planner/blob/831f817e3ddc13ac049eec9d81ec0a81d3d735e3/src/motion_primitives_planner_node.cpp#L206-L238
    
    d. Publish control commands
    https://github.com/hynkis/motion_primitives_planner/blob/831f817e3ddc13ac049eec9d81ec0a81d3d735e3/src/motion_primitives_planner_node.cpp#L77-L91

## Class & Parameters (in motion_primitives_planner_node.hpp)
- Use the class "Node" to manage the position, cost, and collision status of nodes in motion primitives
https://github.com/hynkis/motion_primitives_planner/blob/831f817e3ddc13ac049eec9d81ec0a81d3d735e3/include/motion_primitives_planner/motion_primitives_planner_node.hpp#L58-L96
 
- Change (tune) parameters if you need
https://github.com/hynkis/motion_primitives_planner/blob/831f817e3ddc13ac049eec9d81ec0a81d3d735e3/include/motion_primitives_planner/motion_primitives_planner_node.hpp#L123-L146
    
## Topics to Subscribe 

* /map/local_map/obstacle <-- local cost map

## Topics to Publish 

* /points/selected_motion <-- selected motion primitive for visualization
* /points/motion_primitives <-- motion primitives for visualization
* /car_1/command <-- control command

