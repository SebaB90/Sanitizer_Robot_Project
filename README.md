
# AMR - Sanitizer Robot

## Final Project 

**Students involved in the project:**
- Bertamè Sebastiano - sebastiano.bertame@studio.unibo.it
- Elio Alberto - alberto.elio@studio.unibo.it
- Villari Francesco - francesco.villari@studio.unibo.it


**Instructor:**
- Gianluca Palli - gianluca.palli@unibo.it
- Department: DEI - LAR
- University: University of Bologna
- Address: Viale del Risorgimento 2, 40136 Bologna



##  Task1 - Simulation setup
In order to setup the simulation we used the ROS2 Nav2 package. With the following command from the terminal:

```Bash
export TURTLEBOT3_MODEL=burger

GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models

ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True
```
we exported the robot model 'burger', we initialized the Gazebo world and eventually we started Gazebo and RViz with the TurtleBot3 big_house world.
In order to use the big_house scenario we have modified the configuration file of nav2 to add that specific world. We leave in the repository the file that we have modified.

![BigHouseScenario](https://hackmd.io/_uploads/By5pfY9iT.png)

Moreover, to launch the simulation of the mapping and/or the sanitization we 

## Task 2 - Mapping of the scenario
Following the setup of our simulation environment, we embarked on the mapping task by incorporating the 'slam:=True' parameter into our command line, as shown below. This step is crucial for initiating the SLAM (Simultaneous Localization and Mapping) process:

```Bash
ros2 launch nav2_bringup tb3_simulation_launch.py headless:=True slam:=True
```

In our quest for an effective mapping strategy, we opted for a navigation approach centered around frontier exploration. Our choice was informed by extensive research that highlighted the method's capability for exhaustive environmental coverage and its proficiency in rapidly mapping areas. Frontier exploration stands out for its applicability in environments populated with static obstacles, attributed to its minimal computational requirements and the absence of necessity for intricate adjustments in consistent settings.

For the implementation we used [this repository](https://github.com/SeanReg/nav2_wavefront_frontier_exploration) designed for nav2 compatibility, making it an ideal match for our objectives. Subsequent testing revealed the repository's code to be highly effective in fulfilling our mapping requirements, leading us to adopt this solution for the project's duration.
In the following image the concept of frontier is show.

![Frontier](https://hackmd.io/_uploads/Hk6UjK9sp.png)
<p align="center"><img src=Images/frontier.png/></p>

The code implmented in the repository subscribing to the Occupancy Grid topic to determine known areas of the map. Through nested iterations, it examines each point and its adjacent points to identify frontiers. Once frontiers are established, the algorithm selects a central frontier point and designates a target location for the robot. Utilizing nav2, the robot then plots a course to the specified position. This process is iterative and concludes when no additional frontiers are detected, ensuring thorough mapping of the environment.

The command to launch the exploraiton is the following:

```Bash
ros2 run nav2_wfd explore
```
<p align="center"><img src=Images/mapping.gif alt="animated" /></p>


## Task 3 - Localization in the world 
For the localization we have used the Adaptive Monte Carlo Localization imlemented in the Nav2 package.
This algorithm use a particle filter to localize the robot.

We implemt both a global localization and also a way to give to the monte carlo the initial position.

## Task 4 - Sanitization of the environment
Suppose that:
- In order to kill the coronavirus, the robot is equipped with a set of UV lamps able to spread all around the robot a light power \(P_l = 100 µW/m^2\);
- The UV energy \(E\) at point \((x, y)\) and time \(t\) can be computed as:
  $$
  E(x, y, t) = \int_0^t \frac{P_l}{(x - p_x(\tau))^2 + (y - p_y(\tau))^2} d\tau
  $$
  where \(p_x(t)\) and \(p_y(t)\) represent the robot position along the x and the y axis respectively at time \(t\);
- Any obstacle completely stops the UV power propagation;
- The light power emitted at a distance lower than 0.1 meters from the robot is zero due to the robot encumbrance;
- The room of interest can be discretized as a grid with resolution 0.2 m. 

By sampling the process with sample time \(\Delta t\), the UV energy \(E\) at point \((x, y)\) and sample time \(k\) can be rewritten as:
  $$
  E(x, y, k) = \sum_{i=0}^{k} \frac{P_l \Delta t}{(x - p_x(i \Delta t))^2 + (y - p_y(i \Delta t))^2}
  $$

