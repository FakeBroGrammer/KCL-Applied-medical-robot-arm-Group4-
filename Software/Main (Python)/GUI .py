import sys
import math
import numpy as np
import serial
import serial.tools.list_ports
from PyQt5.QtWidgets import (QApplication, QMainWindow, QWidget, QVBoxLayout, 
                             QHBoxLayout, QLabel, QLineEdit, QPushButton, 
                             QGraphicsScene, QGraphicsView, QGraphicsEllipseItem,
                             QGraphicsLineItem, QMessageBox, QGroupBox, QComboBox)
from PyQt5.QtGui import QPen, QBrush, QColor, QPainter
from PyQt5.QtCore import Qt, QTimer, pyqtSignal, QThread

# ---------------------- 全局放大系数（总放大15倍）----------------------
BASE_SCALE = 10       # 基础放大倍数
EXTRA_SCALE = 1.5     # 额外放大1.5倍
SCALE_FACTOR = BASE_SCALE * EXTRA_SCALE  # 总放大倍数=15

# ---------------------- 工具函数：角度最短路径计算 ----------------------
def normalize_angle(angle_deg):
    """角度归一化到 -180 ~ 180° 范围"""
    angle_deg = math.fmod(angle_deg, 360.0)
    if angle_deg > 180.0:
        angle_deg -= 360.0
    elif angle_deg < -180.0:
        angle_deg += 360.0
    return angle_deg

def shortest_angle_diff(target_deg, current_deg):
    """计算从当前角度到目标角度的最短转动角度（-180~180°）"""
    diff = normalize_angle(target_deg - current_deg)
    return diff

def get_shortest_target_angle(current_deg, target_deg):
    """获取最短路径的目标角度（基于当前角度）"""
    diff = shortest_angle_diff(target_deg, current_deg)
    return current_deg + diff

# ---------------------- 串口通信线程（适配小电机反向）----------------------
class SerialThread(QThread):
    # 定义信号：接收Arduino数据、串口状态变化
    recv_signal = pyqtSignal(float, float, float, float)  # m1当前, m1目标, m2当前, m2目标
    status_signal = pyqtSignal(str)

    def __init__(self, port, baudrate=9600):
        super().__init__()
        self.port = port
        self.baudrate = baudrate
        self.ser = None
        self.is_running = True
        self.target_angles = (0.0, 0.0)  # 待发送的目标角度 (m1, m2)
        self.current_angles = (0.0, 0.0) # 当前角度（用于最短路径计算）
        self.need_send = False

    def run(self):
        try:
            # 打开串口
            self.ser = serial.Serial(self.port, self.baudrate, timeout=0.1)
            self.status_signal.emit(f"串口已连接: {self.port}")
            while self.is_running:
                # 1. 发送目标角度（小电机角度取反 + 最短路径）
                if self.need_send:
                    # 基于当前角度计算最短路径的目标角度
                    m1_short = get_shortest_target_angle(self.current_angles[0], self.target_angles[0])
                    m2_short = get_shortest_target_angle(self.current_angles[1], self.target_angles[1])
                    
                    # 小电机目标角度取反，适配转轴朝下
                    send_m1 = m1_short
                    send_m2 = -m2_short  # 小电机角度反向
                    
                    send_data = f"{send_m1},{send_m2}\n"
                    self.ser.write(send_data.encode('utf-8'))
                    self.need_send = False
                    self.status_signal.emit(f"最短路径发送: m1={m1_short:.1f}°(原{self.target_angles[0]:.1f}°), m2={m2_short:.1f}°(原{self.target_angles[1]:.1f}°)")
                
                # 2. 接收Arduino反馈数据（小电机角度取反）
                if self.ser.in_waiting > 0:
                    try:
                        line = self.ser.readline().decode('utf-8').strip()
                        if line:
                            # 解析Arduino输出格式：m1当前 m1目标 m2当前 m2目标
                            parts = line.split()
                            if len(parts) == 4:
                                m1_current = float(parts[0])
                                m1_target = float(parts[1])
                                # 关键修改：Arduino反馈的小电机角度取反，恢复显示逻辑
                                m2_current = -float(parts[2])  # 小电机当前角度反向
                                m2_target = -float(parts[3])   # 小电机目标角度反向
                                
                                # 更新当前角度（用于下次最短路径计算）
                                self.current_angles = (m1_current, m2_current)
                                self.recv_signal.emit(m1_current, m1_target, m2_current, m2_target)
                    except Exception as e:
                        self.status_signal.emit(f"数据解析错误: {str(e)}")
        except Exception as e:
            self.status_signal.emit(f"串口错误: {str(e)}")
        finally:
            if self.ser and self.ser.is_open:
                self.ser.close()
            self.status_signal.emit("串口已断开")

    def send_target(self, m1_angle, m2_angle):
        """发送目标角度给Arduino（内部自动计算最短路径）"""
        self.target_angles = (m1_angle, m2_angle)
        self.need_send = True

    def stop(self):
        """停止线程"""
        self.is_running = False
        self.wait()

# ---------------------- 机械臂控制主窗口（恢复坐标系数字）----------------------
class TwoLinkArmController(QMainWindow):
    def __init__(self):
        super().__init__()
        # 机械臂核心参数（两根8cm连杆，显示时总放大15倍）
        self.l1 = 8.0 * SCALE_FACTOR  
        self.l2 = 8.0 * SCALE_FACTOR  
        
        # 关节角度（弧度）- 实时更新
        self.theta1 = 0.0  
        self.theta2 = 0.0  
        
        # 末端位置（放大后坐标）- 实时更新
        self.end_x = 0.0
        self.end_y = 0.0
        
        # 串口相关
        self.serial_thread = None
        self.current_m1 = 0.0  # Arduino反馈的m1当前角度
        self.current_m2 = 0.0  # Arduino反馈的m2当前角度（已反向）
        self.target_m1 = 0.0   # Arduino的m1目标角度
        self.target_m2 = 0.0   # Arduino的m2目标角度（已反向）

        # 初始化UI
        self.init_ui()
        # 初始正运动学计算
        self.forward_kinematics()
        # 初始化图形显示
        self.update_arm_display()

    def init_ui(self):
        """初始化图形界面（恢复坐标系数字）"""
        self.setWindowTitle('二连杆机械臂控制器（坐标总放大15倍+刻度数字）')
        self.setGeometry(100, 100, 1600, 1000)  # 主窗口尺寸
        
        # 中心部件
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QHBoxLayout(central_widget)
        main_layout.setSpacing(20)
        main_layout.setContentsMargins(20, 20, 20, 20)
        
        # ---------------------- 左侧控制面板 ----------------------
        control_panel = QWidget()
        control_panel.setFixedWidth(400)
        control_layout = QVBoxLayout(control_panel)
        
        # 1. 串口设置组
        serial_group = QGroupBox('串口设置')
        serial_layout = QHBoxLayout(serial_group)
        
        self.port_combo = QComboBox()
        self.refresh_ports()
        serial_layout.addWidget(QLabel('串口端口:'))
        serial_layout.addWidget(self.port_combo)
        
        refresh_btn = QPushButton('刷新串口')
        refresh_btn.clicked.connect(self.refresh_ports)
        serial_layout.addWidget(refresh_btn)
        
        self.connect_btn = QPushButton('连接串口')
        self.connect_btn.clicked.connect(self.toggle_serial)
        serial_layout.addWidget(self.connect_btn)
        
        # 2. 正运动学控制组
        fk_group = QGroupBox('正运动学 (FK) - 角度→末端位置')
        fk_layout = QVBoxLayout(fk_group)
        
        angle1_layout = QHBoxLayout()
        angle1_layout.addWidget(QLabel('关节1角度 (°):'))
        self.theta1_input = QLineEdit('0')
        self.theta1_input.setPlaceholderText('输入0-360°（支持正负）')
        angle1_layout.addWidget(self.theta1_input)
        
        angle2_layout = QHBoxLayout()
        angle2_layout.addWidget(QLabel('关节2角度 (°):'))
        self.theta2_input = QLineEdit('0')
        self.theta2_input.setPlaceholderText('输入-180-180°（支持正负）')
        angle2_layout.addWidget(self.theta2_input)
        
        self.calc_fk_btn = QPushButton('计算末端位置 (FK)')
        self.calc_fk_btn.clicked.connect(self.calculate_fk)
        fk_layout.addLayout(angle1_layout)
        fk_layout.addLayout(angle2_layout)
        fk_layout.addWidget(self.calc_fk_btn)
        
        self.fk_result_label = QLabel('末端位置: X=0.00 cm, Y=0.00 cm')
        fk_layout.addWidget(self.fk_result_label)
        
        # 3. 逆运动学控制组
        ik_group = QGroupBox('逆运动学 (IK) - 末端位置→角度（支持正负坐标）')
        ik_layout = QVBoxLayout(ik_group)
        
        x_layout = QHBoxLayout()
        x_layout.addWidget(QLabel('目标X (cm):'))
        self.target_x_input = QLineEdit()
        self.target_x_input.setPlaceholderText('输入-16~16 cm（支持正负）')
        x_layout.addWidget(self.target_x_input)
        
        y_layout = QHBoxLayout()
        y_layout.addWidget(QLabel('目标Y (cm):'))
        self.target_y_input = QLineEdit()
        self.target_y_input.setPlaceholderText('输入-16~16 cm（支持正负）')
        y_layout.addWidget(self.target_y_input)
        
        self.calc_ik_btn = QPushButton('计算关节角度 (IK)')
        self.calc_ik_btn.clicked.connect(self.calculate_ik)
        ik_layout.addLayout(x_layout)
        ik_layout.addLayout(y_layout)
        ik_layout.addWidget(self.calc_ik_btn)
        
        self.ik_result_label = QLabel('关节角度: θ1=0.00°, θ2=0.00°')
        ik_layout.addWidget(self.ik_result_label)
        
        # 4. 运动控制按钮组
        motion_group = QWidget()
        motion_layout = QHBoxLayout(motion_group)
        
        self.move_to_ik_btn = QPushButton('运动到IK目标点')
        self.move_to_ik_btn.clicked.connect(self.move_to_ik_target)
        self.move_to_ik_btn.setEnabled(False)
        
        self.move_to_fk_btn = QPushButton('运动到FK角度')
        self.move_to_fk_btn.clicked.connect(self.move_to_fk_angle)
        self.move_to_fk_btn.setEnabled(False)
        
        self.reset_btn = QPushButton('重置机械臂')
        self.reset_btn.clicked.connect(self.reset_arm)
        motion_layout.addWidget(self.move_to_ik_btn)
        motion_layout.addWidget(self.move_to_fk_btn)
        motion_layout.addWidget(self.reset_btn)
        
        # 5. Arduino状态显示组
        status_group = QGroupBox('Arduino实时状态')
        status_layout = QVBoxLayout(status_group)
        
        self.m1_current_label = QLabel('大电机当前角度: 0.00°')
        self.m1_target_label = QLabel('大电机目标角度: 0.00°')
        self.m2_current_label = QLabel('小电机当前角度: 0.00°')
        self.m2_target_label = QLabel('小电机目标角度: 0.00°')
        self.shortest_path_label = QLabel('最短路径提示: 无')
        self.serial_status_label = QLabel('串口状态: 未连接')
        
        status_layout.addWidget(self.m1_current_label)
        status_layout.addWidget(self.m1_target_label)
        status_layout.addWidget(self.m2_current_label)
        status_layout.addWidget(self.m2_target_label)
        status_layout.addWidget(self.shortest_path_label)
        status_layout.addWidget(self.serial_status_label)
        
        # 组装控制面板
        control_layout.addWidget(serial_group)
        control_layout.addWidget(fk_group)
        control_layout.addWidget(ik_group)
        control_layout.addWidget(motion_group)
        control_layout.addWidget(status_group)
        control_layout.addStretch()
        
        # ---------------------- 右侧仿真视图（恢复坐标系数字）----------------------
        # 1. 创建场景（画布范围总放大15倍：-300~300）
        self.scene = QGraphicsScene()
        self.scene.setSceneRect(-300, -300, 600, 600)  # 原-20~20 → 15倍后-300~300
        
        # 2. 创建视图
        self.view = QGraphicsView(self.scene)
        self.view.setRenderHint(QPainter.Antialiasing, True)
        self.view.setMinimumSize(1100, 900)  # 视图尺寸
        
        # 绘制坐标系（总放大15倍）
        # X轴
        x_axis = QGraphicsLineItem(-300, 0, 300, 0)
        x_axis.setPen(QPen(QColor(150, 150, 150), 8))  # 加粗轴线
        self.scene.addItem(x_axis)
        # Y轴
        y_axis = QGraphicsLineItem(0, -300, 0, 300)
        y_axis.setPen(QPen(QColor(150, 150, 150), 8))  # 加粗轴线
        self.scene.addItem(y_axis)
        
        # 刻度标记+数字（每30单位=原2cm，总放大15倍）
        for i in range(-240, 270, 30):
            # 计算原始cm值
            original_cm = i / SCALE_FACTOR
            
            # X轴刻度
            x_tick = QGraphicsLineItem(i, -8, i, 8)
            x_tick.setPen(QPen(QColor(150, 150, 150), 5))
            self.scene.addItem(x_tick)
            # X轴刻度文字（强制显示整数/一位小数，白色字体更醒目）
            x_text = QLabel(f"{original_cm:.0f}" if original_cm.is_integer() else f"{original_cm:.1f}")
            x_text.setStyleSheet("font-size: 28px; font-weight: bold; color: #fff; background: rgba(0,0,0,0.5); padding: 2px 8px; border-radius: 4px;")
            x_text_item = self.scene.addWidget(x_text)
            x_text_item.setPos(i - 20, 15)  # 调整位置避免遮挡
            
            # Y轴刻度
            y_tick = QGraphicsLineItem(-8, i, 8, i)
            y_tick.setPen(QPen(QColor(150, 150, 150), 5))
            self.scene.addItem(y_tick)
            # Y轴刻度文字（强制显示整数/一位小数，白色字体更醒目）
            y_text = QLabel(f"{original_cm:.0f}" if original_cm.is_integer() else f"{original_cm:.1f}")
            y_text.setStyleSheet("font-size: 28px; font-weight: bold; color: #fff; background: rgba(0,0,0,0.5); padding: 2px 8px; border-radius: 4px;")
            y_text_item = self.scene.addWidget(y_text)
            y_text_item.setPos(15, i - 20)  # 调整位置避免遮挡
        
        # 绘制机械臂元素（总放大15倍）
        # 基座（关节1）
        self.base = QGraphicsEllipseItem(-25, -25, 50, 50)
        self.base.setBrush(QBrush(QColor(0, 0, 0)))
        self.scene.addItem(self.base)
        
        # 连杆1
        self.link1 = QGraphicsLineItem()
        self.link1.setPen(QPen(QColor(255, 0, 0), 12))  # 加粗连杆
        self.scene.addItem(self.link1)
        
        # 连杆2
        self.link2 = QGraphicsLineItem()
        self.link2.setPen(QPen(QColor(0, 255, 0), 12))  # 加粗连杆
        self.scene.addItem(self.link2)
        
        # 关节2标记
        self.joint2 = QGraphicsEllipseItem(-20, -20, 40, 40)
        self.joint2.setBrush(QBrush(QColor(128, 128, 128)))
        self.scene.addItem(self.joint2)
        
        # 末端执行器
        self.end_effector = QGraphicsEllipseItem(-20, -20, 40, 40)
        self.end_effector.setBrush(QBrush(QColor(0, 0, 255)))
        self.scene.addItem(self.end_effector)
        
        # IK目标点标记
        self.ik_target_marker = QGraphicsEllipseItem(-25, -25, 50, 50)
        self.ik_target_marker.setBrush(QBrush(QColor(255, 255, 0, 150)))
        self.ik_target_marker.setVisible(False)
        self.scene.addItem(self.ik_target_marker)
        
        # 组装主布局
        main_layout.addWidget(control_panel)
        main_layout.addWidget(self.view)

    # ---------------------- 串口相关方法 ----------------------
    def refresh_ports(self):
        """刷新可用串口列表"""
        self.port_combo.clear()
        ports = serial.tools.list_ports.comports()
        for port in ports:
            self.port_combo.addItem(port.device)

    def toggle_serial(self):
        """连接/断开串口"""
        if self.serial_thread and self.serial_thread.isRunning():
            # 断开串口
            self.serial_thread.stop()
            self.serial_thread = None
            self.connect_btn.setText('连接串口')
            self.move_to_ik_btn.setEnabled(False)
            self.move_to_fk_btn.setEnabled(False)
            self.serial_status_label.setText('串口状态: 未连接')
        else:
            # 连接串口
            if self.port_combo.count() == 0:
                QMessageBox.warning(self, '错误', '未检测到可用串口！')
                return
            port = self.port_combo.currentText()
            self.serial_thread = SerialThread(port, 9600)
            # 绑定信号
            self.serial_thread.recv_signal.connect(self.update_arduino_data)
            self.serial_thread.status_signal.connect(self.update_serial_status)
            self.serial_thread.start()
            self.connect_btn.setText('断开串口')
            self.move_to_ik_btn.setEnabled(True)
            self.move_to_fk_btn.setEnabled(True)

    def update_arduino_data(self, m1_current, m1_target, m2_current, m2_target):
        """更新Arduino反馈的角度数据"""
        self.current_m1 = m1_current
        self.current_m2 = m2_current
        self.target_m1 = m1_target
        self.target_m2 = m2_target
        
        # 更新显示
        self.m1_current_label.setText(f'大电机当前角度: {m1_current:.2f}°')
        self.m1_target_label.setText(f'大电机目标角度: {m1_target:.2f}°')
        self.m2_current_label.setText(f'小电机当前角度: {m2_current:.2f}°')
        self.m2_target_label.setText(f'小电机目标角度: {m2_target:.2f}°')
        
        # 计算最短路径提示
        m1_diff = shortest_angle_diff(m1_target, m1_current)
        m2_diff = shortest_angle_diff(m2_target, m2_current)
        self.shortest_path_label.setText(f'最短路径转动: m1={m1_diff:.1f}°, m2={m2_diff:.1f}°')
        
        # 更新仿真视图
        self.theta1 = math.radians(m1_current)
        self.theta2 = math.radians(m2_current)
        self.forward_kinematics()
        self.update_arm_display()

    def update_serial_status(self, status):
        """更新串口状态显示"""
        self.serial_status_label.setText(f'串口状态: {status}')

    # ---------------------- 正运动学核心算法 ----------------------
    def forward_kinematics(self, theta1=None, theta2=None):
        """正运动学：角度→末端位置（坐标总放大15倍）"""
        t1 = theta1 if theta1 is not None else self.theta1
        t2 = theta2 if theta2 is not None else self.theta2
        
        # 关节2位置（总放大15倍）
        joint2_x = self.l1 * math.cos(t1)
        joint2_y = self.l1 * math.sin(t1)
        
        # 末端位置（总放大15倍）
        end_x = joint2_x + self.l2 * math.cos(t1 + t2)
        end_y = joint2_y + self.l2 * math.sin(t1 + t2)
        
        # 更新全局变量
        if theta1 is not None and theta2 is not None:
            self.theta1 = t1
            self.theta2 = t2
        self.end_x = end_x
        self.end_y = end_y
        
        # 返回原始cm值（用于显示）
        return end_x/SCALE_FACTOR, end_y/SCALE_FACTOR

    # ---------------------- 逆运动学核心算法 ----------------------
    def inverse_kinematics(self, target_x, target_y):
        """逆运动学：正负坐标→角度（输入原始cm值，内部总放大15倍计算）"""
        # 坐标总放大15倍
        target_x_scaled = target_x * SCALE_FACTOR
        target_y_scaled = target_y * SCALE_FACTOR
        
        # 检查工作空间（放大后）
        distance = math.hypot(target_x_scaled, target_y_scaled)
        if distance > self.l1 + self.l2 or distance < abs(self.l1 - self.l2):
            return 0.0, 0.0, False
        
        # 逆解计算（放大后坐标）
        cos_theta2 = (target_x_scaled**2 + target_y_scaled**2 - self.l1**2 - self.l2**2) / (2 * self.l1 * self.l2)
        cos_theta2 = np.clip(cos_theta2, -1.0, 1.0)
        theta2 = math.acos(cos_theta2)
        
        alpha = math.atan2(target_y_scaled, target_x_scaled)
        beta = math.atan2(self.l2 * math.sin(theta2), self.l1 + self.l2 * math.cos(theta2))
        theta1 = alpha - beta
        
        # 角度归一化
        theta1 = normalize_angle(math.degrees(theta1))
        theta2 = normalize_angle(math.degrees(theta2))
        
        return math.radians(theta1), math.radians(theta2), True

    # ---------------------- 正运动学计算触发 ----------------------
    def calculate_fk(self):
        """计算FK：角度→位置（显示原始cm值）"""
        try:
            theta1_deg = float(self.theta1_input.text().strip())
            theta2_deg = float(self.theta2_input.text().strip())
            
            # 角度归一化
            theta1_deg = normalize_angle(theta1_deg)
            theta2_deg = normalize_angle(theta2_deg)
            
            # 更新输入框
            self.theta1_input.setText(f"{theta1_deg:.1f}")
            self.theta2_input.setText(f"{theta2_deg:.1f}")
            
            theta1_rad = math.radians(theta1_deg)
            theta2_rad = math.radians(theta2_deg)
            
            # 执行FK计算（返回原始cm值）
            end_x, end_y = self.forward_kinematics(theta1_rad, theta2_rad)
            
            # 更新显示
            self.fk_result_label.setText(f'末端位置: X={end_x:.2f} cm, Y={end_y:.2f} cm')
            
            # 更新仿真视图
            self.update_arm_display()
            
        except ValueError:
            QMessageBox.warning(self, '输入错误', '请输入有效的数字角度！')

    # ---------------------- 逆运动学计算触发 ----------------------
    def calculate_ik(self):
        """计算IK：正负坐标→角度（输入原始cm值）"""
        try:
            target_x = float(self.target_x_input.text().strip())
            target_y = float(self.target_y_input.text().strip())
            
            # 坐标范围校验
            if not (-16 <= target_x <= 16) or not (-16 <= target_y <= 16):
                QMessageBox.warning(self, '输入错误', '目标坐标需在-16~16cm范围内！')
                return
            
            # 执行IK计算
            theta1_rad, theta2_rad, valid = self.inverse_kinematics(target_x, target_y)
            
            if not valid:
                self.ik_result_label.setText('关节角度: 目标点超出工作空间！')
                QMessageBox.warning(self, '范围错误', f'目标点({target_x},{target_y})超出机械臂工作空间！')
                self.ik_target_marker.setVisible(False)
                return
            
            # 弧度转角度
            theta1_deg = math.degrees(theta1_rad)
            theta2_deg = math.degrees(theta2_rad)
            
            # 更新显示
            self.ik_result_label.setText(f'关节角度: θ1={theta1_deg:.2f}°, θ2={theta2_deg:.2f}°')
            
            # 保存目标角度
            self.ik_target_theta1 = theta1_deg
            self.ik_target_theta2 = theta2_deg
            
            # 显示目标点标记（总放大15倍）
            self.ik_target_marker.setPos(target_x*SCALE_FACTOR, target_y*SCALE_FACTOR)
            self.ik_target_marker.setVisible(True)
            
        except ValueError:
            QMessageBox.warning(self, '输入错误', '请输入有效的数字坐标！')

    # ---------------------- 运动控制 ----------------------
    def move_to_ik_target(self):
        """运动到IK目标点"""
        if not hasattr(self, 'ik_target_theta1'):
            QMessageBox.warning(self, '错误', '请先计算IK角度！')
            return
        if not self.serial_thread or not self.serial_thread.isRunning():
            QMessageBox.warning(self, '错误', '请先连接串口！')
            return
        
        # 发送目标角度
        self.serial_thread.send_target(self.ik_target_theta1, self.ik_target_theta2)

    def move_to_fk_angle(self):
        """运动到FK角度"""
        try:
            theta1_deg = float(self.theta1_input.text().strip())
            theta2_deg = float(self.theta2_input.text().strip())
            
            # 角度归一化
            theta1_deg = normalize_angle(theta1_deg)
            theta2_deg = normalize_angle(theta2_deg)
            
            if not self.serial_thread or not self.serial_thread.isRunning():
                QMessageBox.warning(self, '错误', '请先连接串口！')
                return
            
            # 发送目标角度
            self.serial_thread.send_target(theta1_deg, theta2_deg)
            
        except ValueError:
            QMessageBox.warning(self, '输入错误', '请输入有效的数字角度！')

    # ---------------------- 图形更新 ----------------------
    def update_arm_display(self):
        """更新机械臂仿真视图（坐标总放大15倍）"""
        # 关节2位置（放大后）
        joint2_x = self.l1 * math.cos(self.theta1)
        joint2_y = self.l1 * math.sin(self.theta1)
        
        # 更新连杆（放大后坐标）
        self.link1.setLine(0, 0, joint2_x, joint2_y)
        self.link2.setLine(joint2_x, joint2_y, self.end_x, self.end_y)
        
        # 更新关节和末端位置（中心对齐，放大后）
        self.joint2.setPos(joint2_x - 20, joint2_y - 20)
        self.end_effector.setPos(self.end_x - 20, self.end_y - 20)

    def reset_arm(self):
        """重置机械臂"""
        # 重置输入框
        self.theta1_input.setText('0')
        self.theta2_input.setText('0')
        self.target_x_input.clear()
        self.target_y_input.clear()
        
        # 重置显示
        self.fk_result_label.setText('末端位置: X=0.00 cm, Y=0.00 cm')
        self.ik_result_label.setText('关节角度: θ1=0.00°, θ2=0.00°')
        self.shortest_path_label.setText('最短路径提示: 无')
        self.ik_target_marker.setVisible(False)
        
        # 发送重置角度
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.send_target(0.0, 0.0)
        
        # 重置仿真视图
        self.theta1 = 0.0
        self.theta2 = 0.0
        self.forward_kinematics()
        self.update_arm_display()

    def closeEvent(self, event):
        """窗口关闭时停止串口线程"""
        if self.serial_thread and self.serial_thread.isRunning():
            self.serial_thread.stop()
        event.accept()

# ---------------------- 程序入口 ----------------------
if __name__ == '__main__':
    # 高DPI适配
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps)
    app = QApplication(sys.argv)
    window = TwoLinkArmController()
    window.show()
    sys.exit(app.exec_())