# KCL-Applied-medical-robot-arm-Group4-
7MRI0060 Applied Medical Robotics (25-26) robot arm program,  Group4
This repository contains the full development of Group 4’s medical robotic arm, including CAD design, kinematic modelling, trajectory generation, path planning, PID motor control, UI interface, Arduino firmware, and experiment results.
All code, documentation, and media files required for demonstrating and reproducing our work are included.

* 1. Project Overview
The goal of this project is to design and implement a functional 3-DoF educational robotic arm for the Applied Medical Robotics module.
Our system integrates:
	•	Mechanical design (CAD + fabrication-ready STL files)
	•	Python control framework (IK, planning, GUI)
	•	Arduino motor control firmware (with PID and non-PID variants)
	•	Trajectory tracking and continuous motion execution
	•	A* obstacle-aware path planning
	•	Experiment results for single-point, multi-point, and obstacle avoidance tasks

This project is structured into four major areas:
	1.	CAD – mechanical design and fabrication
	2.	Software – Python control + Arduino firmware
	3.	Docs – system description, control pipeline, demo instructions
	4.	Media(results) – experiments screenshots, simulation and execution results

  ** 2. Repository Structure 
  KCL-Applied-medical-robot-arm-Group4
│
├── CAD/
│   ├── Components and Overall/      
│   ├── Concept/                    
│   └── Iterations/                 
│
├── Docs/
│   ├── control_logic.md             
│   ├── demo_instructions.md          
│   └── system_overview.md           
│
├── Media(results)/
│   ├── continuous points/           
│   ├── obstacles avoidance/         
│   └── single point/                 
│
├── Software/
│   ├── Arduino/
│   │   ├── PID_Last.ino
│   │   ├── large_with_pid.ino
│   │   ├── large_without_pid.ino
│   │   └── small_with_pid.ino        
│   │
│   └── Main (Python)/
│       ├── GUI.py                   
│       ├── kinematics.py            
│       ├── continuous_points.ipynb   
│       ├── single_point.ipynb        
│       └── obstacle_avoid.ipynb      
│
└── README.md

*** 3. CAD Design

The CAD directory contains:

  Components and Overall

Full STL files for:
	•	Arm A
	•	Arm B
	•	Left base
	•	Right base (top / middle / bottom)
	•	Fabrication overview (PNG)

These files represent the final manufacturable design of the robot.

Concept

Early conceptual sketches and first-stage design images:
	•	Arm1.jpg
	•	Arm2.jpg
	•	Base.jpg
	•	Base2.jpg

**** 4. Software Overview

The Software folder includes both Python high-level control and Arduino low-level motor control.



🔹 4.1 Python Control Framework – Software/Main (Python)

Your Python scripts implement:

✔ Kinematics

kinematics.py
	•	Analytical IK
	•	FK verification
	•	Joint-space computation


✔ Control GUI

GUI.py
	•	Real-time control interface
	•	Point input field
	•	Motion execution buttons


✔ Continuous multi-point trajectory

continuous_points.ipynb
	•	Smooth interpolation
	•	Continuous motion generation


✔ A* Path Planning

obstacle_avoid.ipynb
	•	Random environment initialisation
	•	A* shortest path computation
	•	Execution of planned trajectory


✔ Single-point motion

single_point.ipynb
	•	Direct IK + movement demo


🔹 4.2 Arduino Firmware – Software/Arduino

The Arduino code is used to control DC motors and read encoders.

Provided variants include:

✔ small_with_pid.ino

PID-controlled stable low-speed precision motion

✔ large_with_pid.ino

PID for high-speed movement

✔ large_without_pid.ino

Open-loop control for debugging

✔ PID_Last.ino

Final tuned PID implementation

Each firmware reads encoder feedback and controls the motors using PWM.


***** 5. Experimental Results

All experiment screenshots are stored in:

 Media(results)/single point/
	•	Single target input
	•	Motion execution
	•	End-effector position results

 Media(results)/continuous points/
	•	Multiple waypoints
	•	Continuous trajectory demonstration
	•	Simulation vs. real execution

 Media(results)/obstacles avoidance/
	•	A* planning environment
	•	Path solution visualisation
	•	Execution of the planned trajectory

This section provides visual proof of system performance.

****** 6. Control Pipeline

Below is the complete control flow of our system:

Input Target (x, y)
        ↓
Inverse Kinematics (IK)
        ↓
Trajectory Module
    - Single point
    - Multi-point continuous
    - A* obstacle avoidance
        ↓
PID Motor Control (Arduino)
        ↓
Encoder Feedback
        ↓
Python GUI real-time update

******* 7 demo:
video

