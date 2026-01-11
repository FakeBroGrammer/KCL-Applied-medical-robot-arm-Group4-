# Control Logic

## 1. Inverse Kinematics

Inverse kinematics (IK) is used to compute the required joint angles for a given end-effector position in the planar workspace. An analytical IK solution was derived based on the geometric configuration of the two-link arm. Hardware-specific offsets and joint limits are incorporated to ensure feasible solutions.

## 2. Trajectory Generation

To achieve smooth motion, point-to-point commands are interpolated into continuous joint trajectories. Linear interpolation in joint space is applied, ensuring gradual transitions between target angles and avoiding abrupt motor commands.

## 3. PID Control

Each joint is controlled using an independent PID controller running on the Arduino microcontroller. The control law minimizes the error between the target joint angle and the measured angle from the encoder.

The proportional, integral, and derivative gains were manually tuned to balance tracking accuracy, stability, and response speed. PID control was selected due to its simplicity, robustness, and widespread use in robotic joint control applications.

## 4. Motion Planning and Obstacle Awareness

At the task level, motion planning is performed in a discretized planar workspace. A grid-based A* algorithm is used to compute collision-free paths between start and goal positions. The resulting waypoints are converted into continuous trajectories through interpolation.

## 5. User Interface Integration

A Python-based graphical user interface integrates all system components. It allows users to:
- Specify target positions
- Visualize planned paths
- Monitor real-time joint and end-effector trajectories

The GUI acts as the central coordination layer, linking user input, planning, control, and visualization.
