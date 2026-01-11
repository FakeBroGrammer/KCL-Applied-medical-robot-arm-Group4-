import math
import numpy as np
import serial
import serial.tools.list_ports
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread

# ---------------------- Global Scaling Factor (Must Match Kinematics Calculation) ----------------------
BASE_SCALE = 10       # Base magnification factor
EXTRA_SCALE = 1.5     # Additional 1.5x magnification
SCALE_FACTOR = BASE_SCALE * EXTRA_SCALE  # Total magnification = 15x

# ---------------------- Utility Functions: Shortest Angle Path Calculation ----------------------
def normalize_angle(angle_deg):
    """Normalize angle to the range of -180 ~ 180°"""
    angle_deg = math.fmod(angle_deg, 360.0)
    if angle_deg > 180.0:
        angle_deg -= 360.0
    elif angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg

def shortest_angle_diff(target_deg, current_deg):
    """Calculate the shortest rotation angle from current angle to target angle (-180~180°)"""
    diff = normalize_angle(target_deg - current_deg)
    return diff

def get_shortest_target_angle(current_deg, target_deg):
    """Get the target angle for the shortest path based on the current angle"""
    diff = shortest_angle_diff(target_deg, current_deg)
    return current_deg + diff

# ---------------------- Serial Communication Thread (Motion Command Interaction) ----------------------
class SerialThread(QThread):
    # Define signals: receive Arduino data, serial port status change
    recv_signal = pyqtSignal(float, float, float, float)  # m1 current, m1 target, m2 current, m2 target
    status_signal = pyqtSignal(str)

    def __init__(self, port, baudrate=9600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_running = True
        self.target_angles = (0.0, 0.0)  # Target angles to send (m1, m2)
        self.current_angles = (0.0, 0.0) # Current angles (for shortest path calculation)
        self.need_send = False

    def run(self):
        try:
            # Open serial port
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.status_signal.emit(f"Serial port connected: {self.port}")
            while self.is_running:
                # 1. Send target angles (reverse small motor angle + shortest path)
                if self.need_send:
                    # Calculate shortest path target angles based on current angles
                    m1_short = get_shortest_target_angle(self.current_angles[0], self.target_angles[0])
                    m2_short = get_shortest_target_angle(self.current_angles[1], self.target_angles[1])
                    
                    # Reverse small motor target angle to adapt downward rotating shaft
                    send_m1 = m1_short
                    send_m2 = -m2_short  # Reverse small motor angle
                    
                    send_data = f"{send_m1},{send_m2}\n"
                    self.ser.write(send_data.encode('utf-8'))
                    self.need_send = False
                    self.status_signal.emit(f"Shortest path sent: m1={m1_short:.1f}°(orig{self.target_angles[0]:.1f}°), m2={m2_short:.1f}°(orig{self.target_angles[1]:.1f}°)")
                
                # 2. Receive Arduino feedback data (reverse small motor angle)
                if self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline().decode('utf-8').strip()
                        if line:
                            # Parse Arduino output format: m1 current m1 target m2 current m2 target
                            parts = line.split()
                            if len(parts) == 4:
                                m1_current = float(parts[0])
                                m1_target = float(parts[1])
                                # Critical modification: reverse small motor angle from Arduino feedback to restore display logic
                                m2_current = -float(parts[2])  # Reverse current small motor angle
                                m2_target = -float(parts[3])   # Reverse target small motor angle
                                
                                # Update current angles (for next shortest path calculation)
                                self.current_angles = (m1_current, m2_current)
                                self.recv_signal.emit(m1_current, m1_target, m2_current, m2_target)
                    except Exception as e:
                        self.status_signal.emit(f"Data parsing error: {str(e)}")
        except Exception as e:
            self.status_signal.emit(f"Serial port error: {str(e)}")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.status_signal.emit("Serial port disconnected")

    def send_target(self, m1_angle, m2_angle):
        """Send target angles to Arduino (shortest path calculated automatically)"""
        self.target_angles = (m1_angle, m2_angle)
        self.need_send = True

    def stop(self):
        """Stop the thread"""
        self.is_running = False
        self.wait()

# ---------------------- Kinematics Core Algorithm (Forward/Inverse Solutions) ----------------------
class ArmKinematicsCore:
    def __init__(self, l1=8.0, l2=8.0, scale_factor=SCALE_FACTOR):
        self.l1 = l1 * scale_factor  # Link 1 length (scaled)
        self.l2 = l2 * scale_factor  # Link 2 length (scaled)
        self.scale_factor = scale_factor  # Scaling factor

    def forward_kinematics(self, theta1_rad, theta2_rad):
        """Forward Kinematics: Angles → End-effector position (return scaled coordinates + original cm coordinates)"""
        # Joint 2 position (scaled)
        joint2_x = self.l1 * math.cos(theta1_rad)
        joint2_y = self.l1 * math.sin(theta1_rad)
        
        # End-effector position (scaled)
        end_x = joint2_x + self.l2 * math.cos(theta1_rad + theta2_rad)
        end_y = joint2_y + self.l2 * math.sin(theta1_rad + theta2_rad)
        
        # Convert to original cm coordinates
        end_x_cm = end_x / self.scale_factor
        end_y_cm = end_y / self.scale_factor
        
        return (end_x, end_y), (end_x_cm, end_y_cm)

    def inverse_kinematics(self, target_x_cm, target_y_cm):
        """Inverse Kinematics: Original cm coordinates → Angles (radians), return (theta1_rad, theta2_rad, validity)"""
        # Scale coordinates
        target_x_scaled = target_x_cm * self.scale_factor
        target_y_scaled = target_y_cm * self.scale_factor
        
        # Workspace validation (scaled)
        distance = math.hypot(target_x_scaled, target_y_scaled)
        if distance > self.l1 + self.l2 or distance < abs(self.l1 - self.l2):
            return 0.0, 0.0, False
        
        # Inverse solution calculation (scaled coordinates)
        cos_theta2 = (target_x_scaled**2 + target_y_scaled**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
        theta2 = math.acos(cos_theta2)
        
        alpha = math.atan2(target_y_scaled, target_x_scaled)
        beta = math.atan2(self.l2 * math.sin(theta2), self.l1 + self.l2 * math.cos(theta2))
        theta1 = alpha - beta
        
        # Normalize angles (convert to degrees first then back to radians to ensure range)
        theta1_deg = normalize_angle(math.degrees(theta1))
        theta2_deg = normalize_angle(math.degrees(theta2))
        
        return math.radians(theta1_deg), math.radians(theta2_deg), True

# ---------------------- Independent Test Logic ----------------------
if __name__ == "__main__":
    # Test 1: Forward Kinematics Calculation
    print("=== Forward Kinematics Test ===")
    kinematics = ArmKinematicsCore()
    theta1_rad = math.radians(30)  # 30°
    theta2_rad = math.radians(-45) # -45°
    scaled_pos, cm_pos = kinematics.forward_kinematics(theta1_rad, theta2_rad)
    print(f"Input Angles: θ1=30°, θ2=-45°")
    print(f"Scaled Coordinates: X={scaled_pos[0]:.2f}, Y={scaled_pos[1]:.2f}")
    print(f"Original cm Coordinates: X={cm_pos[0]:.2f}cm, Y={cm_pos[1]:.2f}cm\n")

    # Test 2: Inverse Kinematics Calculation
    print("=== Inverse Kinematics Test ===")
    target_x = 10.0  # 10cm
    target_y = 5.0   # 5cm
    theta1_rad, theta2_rad, valid = kinematics.inverse_kinematics(target_x, target_y)
    if valid:
        theta1_deg = math.degrees(theta1_rad)
        theta2_deg = math.degrees(theta2_rad)
        print(f"Target Coordinates: X={target_x}cm, Y={target_y}cm")
        print(f"Calculated Angles: θ1={theta1_deg:.2f}°, θ2={theta2_deg:.2f}°")
    else:
        print(f"Target coordinates ({target_x}cm, {target_y}cm) are outside the workspace!")
    
    # Test 3: Shortest Angle Path Calculation
    print("\n=== Shortest Angle Path Test ===")
    current_deg = 170.0
    target_deg = -170.0
    diff = shortest_angle_diff(target_deg, current_deg)
    shortest_target = get_shortest_target_angle(current_deg, target_deg)
    print(f"Current Angle: {current_deg}°, Target Angle: {target_deg}°")
    print(f"Shortest Rotation Angle: {diff}°, Actual Target Angle: {shortest_target}°")

    # Test 4: Serial Port List Query (Runnable without hardware)
    print("\n=== Serial Port List Test ===")
    ports = serial.tools.list_ports.comports()
    if ports:
        print("Available Serial Ports:")
        for port in ports:
            print(f"  - {port.device} ({port.description})")
    else:
        print("No available serial ports detected!")