# Final Project

## Autonomous and Mobile Robotics M

**Instructor:**
- **Name:** Gianluca Palli
- **Email:** gianluca.palli@unibo.it
- **Department:** DEI - LAR
- **University:** University of Bologna
- **Address:** Viale del Risorgimento 2, 40136 Bologna

## Sanitizer Robot

### Task 1
The students are asked to:
- Setup the simulation of the TurtleBot3 burger robot using the Gazebo simulation environment and the TurtleBot3 Big House scenario. Note that during the exam evaluation the robot can be spawned on the simulation scenario in a random position and additional obstacles can be present;
- OR setup the real TurtleBot3 robot and make it working in a confined environment with obstacles. Note that the shape and size of the environment will be not known in advance.

### Task 2
The students are asked to make the robot able to move autonomously in the environment to create a map covering the larger possible part of the environment:
- No previous knowledge of the map will be considered during the exam;
- Existing packages can be used;
- Implementation (or learning) of (sub)optimal map generation processes will be positively evaluated.

### Task 3
After the mapping of the scenario (Task 2) the students are asked to makes the robot able to navigate in the environment reaching a set of goals contained in a text file:
- The alignment between the map and the navigation system must be performed;
- Implementation (or learning) of (sub)optimal map alignment processes will be positively evaluated.

### Task 4
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

### General Rules
The students are asked to address the following tasks using ROS the TurtleBot3 robot moving in an unknown environment:
- The tasks can be developed either in simulation or with the real robot;
- A report (presentation or document) must be presented at the exam together with the implemented code (provided separately not in the report);
- During the exams (project presentation), the students will be asked to perform show the results by means of simulations, experiments with the real robot, or videos;
- A clear organization of the project and the related code is highly recommended;
- Available ROS packages can be exploited for the project implementation.

### Task 4 Implementation Example
- ROS environment with simulation of Turtlebot3 Burger robot running on gazebo big house
- Simulation services and actions include Lamps, Energy control, Task parameter update, Sanitization.

#### Task 4 Implementation Example (Continued)
- Path generation is divided in two steps:
  - First, a path covering the room is defined on the base of the room shape.
  - Second, the path is adjusted to move around obstacles, searching for the closest free point to the path in the obstacle neighborhood.
- Grid maps are used to visualize the results.

---

