
# Sanitizer Robot Project

**Group that realized the project:**
- Bertamè Sebastiano - sebastiano.bertame@studio.unibo.it
- Elio Alberto - alberto.elio@studio.unibo.it
- Villari Francesco - francesco.villari@studio.unibo.it


##  Task1 - Simulation setup
In order to setup the simulation we used the ROS2 Nav2 package. With the following command from the terminal:

```Bash
export TURTLEBOT3_MODEL=burger

GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False
```
we exported the robot model 'burger', we initialized the Gazebo world and eventually we started Gazebo and RViz with the TurtleBot3 big_house world.
In order to use the big_house scenario we modified the configuration file of nav2 to add that specific world. The modified files are uploaded in the GitHub repository.

![bighouse](https://github.com/AlbyElio/AMR_SanitizerRobotProject/assets/151182760/a512c2ce-dc91-45dd-a65c-3df146bb8e05)


To launch the simulation of the mapping and/or the sanitization we implemented a system that allows to pass some arguments according to the tasks that the user wants to be completed. When the simulation is launched, the user is asked to provide some parameters, that are:
- the x and y coordinates of the initial pose of the robot
- the task to perform: enter 0 for the mapping or 1 for the sanitization
- the localization method to be used: 0 for imposing as initial estimation for the Adaptive Monte Carlo Localization method the pose given by the user or 1 to use the global localization service of this method

We decided to split the simulation launch in two different launch files. The first one is in charge to start the TurtleBot3 simulation inside the package nav2_bringup that starts Gazebo and RViz with the world set in the parameters files (in our case we changed it in big_house) and asks for the parameters metioned above. The parameters passed by the user on the terminal are then written in a text file and are read by the second launch file. This last, starts all the specitìfic nodes that have been created to perform all the tasks of the project. The nodes are described below in the next sections.

## Task 2 - Mapping of the scenario
Following the setup of our simulation, we embarked on the mapping task by incorporating the 'slam:=True' parameter into our command line, as shown below. This step is crucial for initiating the SLAM (Simultaneous Localization and Mapping) process:

```Bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=False slam:=True
```

In our quest for an effective mapping strategy, we opted for a navigation approach based on **frontier exploration**. Our choice was driven by the strength of this method, that are its capability to achieve an exhaustive environmental coverage and its proficiency in rapidly mapping areas. Frontier exploration stands out for its applicability in environments populated with static obstacles, attributed to its minimal computational requirements and the absence of necessity for intricate adjustments in consistent settings.

For the implementation we used [this repository](https://github.com/SeanReg/nav2_wavefront_frontier_exploration) designed for nav2 compatibility, making it an ideal match for our objectives. Subsequent testing revealed the repository's code to be highly effective in fulfilling our mapping requirements, leading us to adopt this solution for the project's duration.
In the following image the concept of frontier is shown.


![Frontier](https://hackmd.io/_uploads/Hk6UjK9sp.png)

The code implmented in the repository subscribes to the *Global Occupancy Grid topic* to determine known areas of the map. Through nested iterations, it examines each point and its adjacent points to identify frontiers. Once frontiers are established, the algorithm selects the central point of the furthest frontier and designates a target location goal for the robot. Utilizing nav2, the robot then plots a path to the specified position. This process is iterative and concludes when no additional frontiers are detected, ensuring a thorough mapping of the environment.

The command to launch the exploraiton is the following:

```Bash
ros2 run nav2_wfd explore
```
The main problem we faced on during the implementation of this  method has been the lackness of frontiers even if the map was not fully complete.
In fact, due to the specific lidar model and mapping method of 'bruger' device, it is not allowed to mark an area as free space in no obstacles are seen. This means that in big empty spaces (like *turtlebot empty world*) 'burger' is not capable to achieve a good mapping result.
The image below shows the most frequent instance of this problem we ecountered during developing the mapping task:

![frontiere](https://hackmd.io/_uploads/ByMSs3oi6.png)

When escaping from a room, if the distance to the opposite wall is larger than the laser range there is no chance to mark as free space any point in the second room. This means that no frontiers can be generated.

The only way to solve this problem without making changes in the mapping algorithm is providing a lidar device with larger maximum range.

https://github.com/AlbyElio/AMR_SanitizerRobotProject/assets/151182760/0a0650db-33a7-4ca7-a109-a0bdf2bd49dc

## Task 3 - Localization in the world 
For the localization we have used the Adaptive Monte Carlo Localization imlemented in the Nav2 package. To localize the robot, this algorithm uses a particle filter, a localization method based on the continuous update of the probability of the robot to be positioned in a specific point of the map. The algorithm starts with the initialization of a high number of points/particles (e.g. 100) uniformly spawned on the map. Then, the following steps are iteratively performed:

- the weights of the particles are updated basing on how much the position on of the point in the map matches the sensor reading. Namely, if the obstacles near the point are the same as the ones seen by the LIDAR the weight of the point increases.
- the covariance matrix is updated. This matrix shows the variance between the estimations of the spacial and angular coordinates of the robot.
- a resample of the particles is executed in order to relocate the points close to the ones with the higher weights.

The position estimation is performed for the entire duration of the simulation.
As mentioned in the task 1 Section, to determine the initial position of the robot we implemented two different methods. With the first one, we impose the initial pose as the chosen by the user so the covariance will be immediately very low and the robot will soon start to move. With the second approach, we initialize the global localization service obtaining a cloud of particles randomly spread in the map. Then our node, called init_localization_node, makes the robot to start spinning for a given amount of time and then to move to a random waypoint set at a distance of 0.55 meters.


https://github.com/AlbyElio/AMR_SanitizerRobotProject/assets/151182760/eb879dd2-37ac-4dc4-86b8-64a156305b57

## Task 4 - Sanitization of the environment
To sanitize the envirnoment we discretized the map with a resolution of 0.2 meters and applied the following energy distribution law:
$$E(x, y, t) = \int_0^t \frac{P_l}{(x - p_x(\tau))^2 + (y - p_y(\tau))^2} d\tau$$
To implement it in the code we used the discrtized equation that is:
$$E(x, y, k) = \sum_{i=0}^{k} \frac{P_l \Delta t}{(x - p_x(i \Delta t))^2 + (y - p_y(i \Delta t))^2}$$

The sanitization process has been divided in three steps:
- UV power evaluation: for every point of the discretized map we computed the related instant power value using the node power_publisher_node. This latter, computes the power according to the distance from the robot and shape of the room, provided by the /laser_scan topic. With the message of the laser scan, we generate a polygon using as vertices the points in which the laser bumps into. The power is then evaluated only for the point of the map inside the polygon and it's set as 0 for the points outside the polygon. Eventually, the power is published as a message of type OccupancyGrid via our custom topic /power_map. This process is shown in the figure below.
- Energy evaluation: the topic published by the previous node is subscribed by the node /energy_publisher_node and integrated using the formula above. The sampling time $\Delta t$ is given by measuring the interval of time between two consecutive publications of the power message. Also he energy is published as an OccupancyGrid message via the topic /energy_map.
- Navigation: the node /energy_navigation_node receives the map of the energy published in the previous step and computes a waypoint to be reached using the Navigate To Pose action server given by the Nav2 ROS2 package. The waypoint is chosen as the closest point in the map with a energy level lower than 10mJ among the ones in the desired rooms to be sanitized.

For the RViz visualization we choosed the cost_map color scheme that imposes to use values in the range 0 to 98. In order to cover the entire range, we converted the values of the maps by multiplying by a scaling factor.

![UV_penetration](https://github.com/AlbyElio/AMR_SanitizerRobotProject/assets/151182760/f0832033-6a49-4a9b-8d2a-06cc61410676)


https://github.com/AlbyElio/AMR_SanitizerRobotProject/assets/151182760/7c7eac71-2844-4ace-b5ae-005f37467ff0

## Practical implementation

### Navigation setup
As shown in the figure below, when the robot is navigating toward a goal pose and an obstacle is present between the goal the robot itself, the local planner tries to minimize a cost function that depends on several parameters. In our case the burger model uses DWB Controller as the local planner and the parameters of the cost function are defined in [here](https://navigation.ros.org/configuration/packages/configuring-dwb-controller.html). 

![80535726-e4954200-89be-11ea-86b7-1908a39f0e89](https://github.com/AlbyElio/AMR_SanitizerRobotProject/assets/151182760/247b9749-abdc-4a0d-b05d-8beccdf7a8ff)

As explained in [this forum](https://github.com/ros-planning/navigation2/issues/938) by one of the main contributors of the Nav2 package, in situations as the one shown in the image the weights of the parameters GoalAlign.scale and GoalDist.scale set cost of the path generated by the global planner to a very high value. The result is that the robot does't circumvent the obstacle but, instead, attemps to go through it. Since there is an obstacle avoidance policy, the robot doesn't crash into the obstacle but stalls and, to partially solve the problem, we decreased the default weights of the mentioned parameters.
To improve more the navigation, as explained in [this article](https://arxiv.org/pdf/1706.09068.pdf) in section 3.1.2, we tuned the sim_time parameter of the local planner by increasing it to a value of 5 seconds to ensure a sufficiently long time interval to obtain the optimal trajectory minimizing the cost function previously mentioned.

Another parameter that we needed to change is the local cost map inflation factor. This parameter performs a virtual enlargement of the obstacles for safety reasons. Since there are several narrow spaces in the used world, the navigation was compromised and we looked for a trade-off between collision avoidance and smooth navigation. It can meet some problems with a specific kind of table which foot base is very large but it's too low to be seen by the LIDAR. Moreover, we changed the update frequency of the local and global cost maps to increase the probability to map a portion of the world during the frontier exploration.

### LIDAR maximum range and frequency
We need to change following two parameters of the LIDAR, the scan frequeny and the maximum range because they provided some issues in two different scenarios. The first scenario was the recognition of the free spaces in the SLAM operation as explained in the Task 1 section. The second one was the computation of the power cost map. In fact, a low maximum range of the LIDAR does not allow us to generate the polygon used to compute the points to be sanitized. This implies that there was no way to avoid the UV rays to go through the walls or other obstacles positioned further than the maximum range. For what concernes the frequency of the laser topic publication, a low frequency update lead to a slow update of the shape of the polygon, causing in its turn a delay in the power map update with respect to the actual robot pose.
In order to avoid this problems we modified the two parameter increasing both of them. We also found a cheap [commercial LIDAR](https://www.robot-italy.com/it/rplidar-a1m8-360-degree-laser-scanner.html) that matches our performace requests.

### Goal and path tolerances
The SLAM toolbox offers the possibility to set a tolerance on the final pose and on the path following. Since we don't need an high precision on reaching on the final pose for these specific tasks, we set high tolerance values.

### Sanitization
Making a simple calculation, we concluded that to completely sanitize a point in the map 1 meter far from the robot it would have taken 100 seconds. To speed up the simulation we decided to multiply the value of the instantaneous power by a factor 5 during the integraion for the energy evaluation.

### Easy user-interface
To make the program easy to use by a non-expert user we decided to implement a simple user interface that allow him/her to easily choose the room to be sanitized. The interface is shown below.
![UserInterface](https://github.com/AlbyElio/AMR_SanitizerRobotProject/assets/151182760/e8e1eae6-b435-49f0-b944-7e61ca41b966)


