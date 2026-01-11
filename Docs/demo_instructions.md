# Demo Instructions

## 1. Hardware Setup

1. Connect the robotic arm motors and encoders to the Arduino according to the wiring diagram.
2. Power the motors using an external power supply.
3. Connect the Arduino to the host computer via USB.

## 2. Software Requirements

- Python 3.x
- Required Python libraries (e.g., numpy, matplotlib, pyserial)
- Arduino IDE

## 3. Running the Demo

1. Upload the Arduino control code to the microcontroller.
2. Open the Python GUI script on the host computer.
3. Select the correct serial port corresponding to the Arduino.
4. Launch the GUI interface.

## 4. Testing the Robotic Arm

- **Point-to-point test:**  
  Enter a target Cartesian coordinate and observe the arm moving to the desired position.

- **Trajectory tracking test:**  
  Specify a sequence of waypoints and verify smooth continuous motion.

- **Visualization check:**  
  Compare the planned trajectory with the real-time end-effector path shown in the GUI.

## 5. Expected Results

The robotic arm should demonstrate stable joint motion, accurate positioning, and consistent feedback visualization. Minor deviations may occur due to mechanical tolerances and actuator limitations.
