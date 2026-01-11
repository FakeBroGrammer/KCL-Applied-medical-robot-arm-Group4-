# System Overview

## 1. Hardware Architecture

The robotic system consists of a two-degree-of-freedom planar robotic arm designed as a low-cost and modular experimental platform. The mechanical structure was modeled using CAD software and fabricated via 3D printing. Two DC geared motors equipped with incremental encoders are used to actuate the joints, providing angular position feedback for closed-loop control.

An Arduino microcontroller serves as the low-level control unit. It interfaces directly with the motors and encoders, executes PID control loops at a fixed sampling rate, and communicates with a host computer via serial communication.

## 2. Software Architecture

The software system follows a layered architecture:

- **Low-level layer (Arduino):**  
  Responsible for encoder reading, PID-based joint control, and motor command execution.

- **High-level layer (PC):**  
  Implemented in Python, this layer handles inverse kinematics computation, trajectory interpolation, motion planning, and user interaction through a graphical user interface (GUI).

- **Communication layer:**  
  A serial communication protocol is used to transmit target joint angles and receive real-time joint feedback.

## 3. System Workflow

1. The user specifies a target position through the GUI.
2. The desired end-effector position is converted into joint angles using inverse kinematics.
3. Trajectory interpolation generates smooth joint references.
4. Target joint angles are sent to the Arduino controller.
5. The PID controller drives the motors to track the reference angles.
6. Encoder feedback is transmitted back to the PC for visualization and validation.

This workflow reflects a simplified but representative pipeline commonly used in medical robotic systems for image-guided and goal-directed motion.
