# 3D-Trajectory-Optimization-Collision-Avoidance-for-Autonomous-Quadrotors
This repository showcases the advanced integration of graph search algorithms, trajectory optimization, and dynamic collision management for an autonomous quadrotor navigating through complex 3D environments. Explore the intricate balance between computational efficiency and real-world robotic adaptability.

![quad_control_maze_1_3](https://github.com/Saibernard/3D-Trajectory-Optimization-Collision-Avoidance-for-Autonomous-Quadrotors/assets/112599512/91f07a88-de45-4f85-ab91-e79b71a800aa)

## Overview

### Autonomous Navigation for Quadrotors in 3D Environments
Autonomous navigation in 3D environments is a challenging blend of robotics, computational mathematics, and adaptive systems. This project dives deep into a simulated quadrotor's prowess to traverse a complex, obstacle-ridden environment. Leveraging graph search algorithms, trajectory smoothing methods, and dynamic collision strategies, this venture is a deep dive into the nuances of robotic precision, control, and adaptability.

## Enhanced Quadrotor Model
Simulated as a sphere with a 0.25m radius, our quadrotor exhibits distinct maneuverability in a 3D space. This model's selection marries aerodynamic attributes with motion dynamics. Together with traits from Phase 1, the quadrotor is poised to manage various torque and thrust conditions, crucial during waypoint transitions and sudden trajectory alterations.

## Advanced Trajectory Generation

### Graph Search Algorithms
Our quadrotor utilizes the Dijkstra and A* algorithms for the shortest path determination. However, the voxel grid-based discretization can sometimes yield paths with sharp directional shifts, suboptimal for real-world trajectories.

### Post-Processing & Trajectory Smoothing
To tackle the abruptness of direct paths, we introduce a trajectory smoothing component. It not only economizes on energy by eliminating redundant waypoints but strikes a harmony between the shortest and most practical paths.

### Polynomial Interpolations
Polynomial curves are pivotal in assuring a path that is not just minimal in length but also fluent. Leveraging minimum jerk or snap polynomial segments, the quadrotor seamlessly handles tight turns with minimal acceleration variance.

## Collision Management & Environmental Interactions

### Dynamic Margin Parameter
A dynamically adjustable margin parameter provides the quadrotor with a cushion against obstacles. This safety buffer optimizes the quadrotor’s trajectory without jeopardizing its protection.

### Grid Resolution Fine-Tuning
Grid resolution dramatically influences quadrotor efficiency. While a finer grid can reveal hidden shortcuts, it could also escalate computational loads.

## Code Integration & Execution

### Standardized Code Framework
A standardized project packet promotes uniformity, simplifying the debugging chore and guaranteeing consistent outcomes across diverse simulations.

### WorldTraj Class Essence
As the linchpin of trajectory strategizing, this class amalgamates waypoints and control parameters from preceding phases. It dictates the quadrotor's real-time trajectory, factoring in dynamic conditions.

### Iterative Testing Approach
The `util/test.py` tool persistently evaluates the quadrotor's navigational prowess. Testing it against a variety of maps, this iterative strategy refines algorithms to accommodate an assortment of challenges.

![3D_Path](https://github.com/Saibernard/3D-Trajectory-Optimization-Collision-Avoidance-for-Autonomous-Quadrotors/assets/112599512/9bf0e7db-a249-4b39-bc51-0efb2453c6cc)

![A_Path,_Waypoints,_and_Trajectory](https://github.com/Saibernard/3D-Trajectory-Optimization-Collision-Avoidance-for-Autonomous-Quadrotors/assets/112599512/11672668-a4d8-4143-9ad0-fa531fedf012)

![Position_vs_Time](https://github.com/Saibernard/3D-Trajectory-Optimization-Collision-Avoidance-for-Autonomous-Quadrotors/assets/112599512/7247c5cb-657a-4cfa-945c-b77e6b1a2fc4)

![Orientation_vs_Time](https://github.com/Saibernard/3D-Trajectory-Optimization-Collision-Avoidance-for-Autonomous-Quadrotors/assets/112599512/75d1bb44-04ad-46b8-ba41-b1a1634f8e2d)

![Commands_vs_Time](https://github.com/Saibernard/3D-Trajectory-Optimization-Collision-Avoidance-for-Autonomous-Quadrotors/assets/112599512/8257a815-1b5c-4cfd-90bc-bd4cf5eabef7)


---

Embarking on this project is more than a journey through 3D navigation. It's a testament to the modern challenges of robotics. The delicate interplay of computation, mechanics, and unpredictable landscapes encapsulates the obstacles and achievements of autonomous mechanisms.
