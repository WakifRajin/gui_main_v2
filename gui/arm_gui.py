'''
CHANGED CONTROLS OVERVIEW (pls check with gamepad and remap if the buttons dont work accordingly)

Base Motor: Left stick
  Left : Base forward
  Right : Base backward
  Centered: Base stop

Shoulder Motor: 
  Y: Shoulder up (forward)
  A: Shoulder down (backward)
  Neither pressed: Shoulder stop

Elbow Motor: Left stick
  Up : Elbow up (forward)
  Down : Elbow down (backward)
  Centered: Elbow stop

(Kept all other gamepad mappings the same as before)

RT/LT: Adjust shared PWM
RB/LB: Adjust wrist servo
D-pad: Control gripper and roller
L/R buttons: Stop gripper/roller

START: Reset all controls

'''

import sys
import json
import numpy as np
import threading
import logging
import pygame
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from PyQt5.QtWidgets import (
    QApplication, QWidget, QVBoxLayout, QHBoxLayout, QLabel,
    QPushButton, QSlider, QGroupBox, QMainWindow, QButtonGroup
)
from PyQt5.QtCore import Qt, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure

# Configure logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

# Arm segment lengths
L1, L2, L3 = 10, 10, 5

class ArmCommandPublisher(Node):
    def __init__(self):
        super().__init__('arm_control_gui')
        self.publisher_ = self.create_publisher(String, '/arm_command', 10)
        self.get_logger().info('ROS2 Arm Command Publisher initialized')
        
    def publish_command(self, command):
        msg = String()
        msg.data = json.dumps(command)
        self.publisher_.publish(msg)
        self.get_logger().info(f'{msg.data}')

class DetachedPlotWindow(QMainWindow):
    def __init__(self, gui):
        super().__init__()
        self.gui = gui
        self.setWindowTitle("Detached Plot")
        self.setGeometry(300, 300, 600, 600)
        self.setCentralWidget(gui.canvas)

    def closeEvent(self, event):
        self.gui.reattach_plot()
        event.accept()

class ArmControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Arm Control GUI")
        self.setGeometry(100, 100, 800, 700)

        # Initialize pygame for gamepad support
        pygame.init()
        pygame.joystick.init()
        
        # Check for gamepads
        self.joystick = None
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            logger.info(f"Gamepad connected: {self.joystick.get_name()}")
        else:
            logger.warning("No gamepad detected")

        # Control variables
        self.gripper_state = 0  # 0: stop, 1: close, 2: open
        self.roller_state = 0   # 0: stop, 1: close, 2: open
        self.servo_angle = 90
        self.elbow_pwm = 0
        self.shoulder_pwm = 0
        self.base_pwm = 0
        self.shared_pwm = 0
        self.last_values = None

        # Motor states (0: stop, 1: forward, 2: backward)
        self.base_state = 0
        self.shoulder_state = 0
        self.elbow_state = 0

        # ROS2 setup
        rclpy.init()
        self.ros_node = ArmCommandPublisher()
        self.ros_thread = threading.Thread(target=self.ros_spin, daemon=True)
        self.ros_thread.start()

        # GUI setup
        self.detached_window = None
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.main_layout = QHBoxLayout(self.main_widget)

        self.init_controls()
        self.init_plot()

        # Timer for updates
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_output)
        self.timer.start(50)  # Update every 50ms
        
        # Timer for gamepad updates
        self.gamepad_timer = QTimer()
        self.gamepad_timer.timeout.connect(self.update_gamepad)
        self.gamepad_timer.start(100)  # Update every 100ms
        
        # Gamepad button state tracking
        self.prev_buttons = [False] * 15  # Track previous button states
        self.hat_state = (0, 0)  # Track D-pad state
        self.last_gamepad_update = pygame.time.get_ticks()
        
        # Gamepad axis deadzone
        self.deadzone = 0.2

    def ros_spin(self):
        """Spin the ROS node in a separate thread"""
        rclpy.spin(self.ros_node)
        self.ros_node.destroy_node()
        rclpy.shutdown()

    def send_ros_message(self, message):
        """Send message through ROS2 publisher"""
        try:
            self.ros_node.publish_command(message)
            return True
        except Exception as e:
            logger.error(f"Failed to send ROS message: {e}")
            return False

    def init_controls(self):
        control_panel = QVBoxLayout()
        control_panel.setAlignment(Qt.AlignTop)

        # Add ROS2 connection status
        self.status_label = QLabel("🟢 ROS Connected")
        self.status_label.setStyleSheet("font-weight: bold; padding: 5px; color: green;")
        control_panel.addWidget(self.status_label)
        
        # Gamepad status
        self.gamepad_status = QLabel("🔴 No gamepad")
        self.gamepad_status.setStyleSheet("font-weight: bold; padding: 5px;")
        control_panel.addWidget(self.gamepad_status)

        # Shared PWM slider for base, shoulder, and elbow motors
        self.shared_pwm_slider = self.create_pwm_slider("Shared Motor PWM", lambda val: setattr(self, 'shared_pwm', val), 0, 1023)
        control_panel.addWidget(self.shared_pwm_slider)
        
        # Motor control sections
        control_panel.addWidget(self.create_motor_control("Base Motor", 'base'))
        control_panel.addWidget(self.create_motor_control("Shoulder Motor", 'shoulder'))
        control_panel.addWidget(self.create_motor_control("Elbow Motor", 'elbow'))
        
        # Servo control
        self.servo_slider = self.create_pwm_slider("Wrist Servo (0-180°)", lambda val: setattr(self, 'servo_angle', val), 0, 180)
        control_panel.addWidget(self.servo_slider)
        
        # Gripper and Roller controls
        control_panel.addWidget(self.create_gripper_roller_control("Gripper", 'gripper'))
        control_panel.addWidget(self.create_gripper_roller_control("Roller", 'roller'))

        btn_layout = QHBoxLayout()
        reset_btn = QPushButton("Reset")
        reset_btn.setMinimumHeight(40)
        reset_btn.clicked.connect(self.reset_all)

        self.detach_btn = QPushButton("Detach Plot")
        self.detach_btn.setMinimumHeight(40)
        self.detach_btn.clicked.connect(self.toggle_plot_detach)

        btn_layout.addWidget(reset_btn)
        btn_layout.addWidget(self.detach_btn)
        control_panel.addLayout(btn_layout)

        wrapper = QWidget()
        wrapper.setLayout(control_panel)
        wrapper.setStyleSheet("""
            QWidget { background-color: #f4f4f4; font-family: 'Segoe UI', Arial; font-size: 12pt; }
            QGroupBox { border: 1px solid #cccccc; border-radius: 8px; margin-top: 1.5ex; padding: 10px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QPushButton { background-color: #2b2d42; color: white; border-radius: 8px; padding: 8px; font-weight: bold; }
            QPushButton:hover { background-color: #1f2235; }
            QPushButton:checked { background-color: #4a4d6d; }
            QSlider::groove:horizontal { background: #cccccc; height: 10px; border-radius: 5px; }
            QSlider::handle:horizontal { background: #2b2d42; width: 30px; height: 30px; border-radius: 15px; margin: -10px 0; }
        """)
        self.main_layout.addWidget(wrapper, 4)

    def init_plot(self):
        self.figure = Figure(figsize=(6, 6))
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.main_layout.addWidget(self.canvas, 5)

    def reattach_plot(self):
        if self.detached_window:
            self.detached_window.close()
            self.detached_window = None
            self.main_layout.addWidget(self.canvas, 5)
            self.detach_btn.setText("Detach Plot")

    def toggle_plot_detach(self):
        if self.detached_window:
            self.reattach_plot()
        else:
            self.main_layout.removeWidget(self.canvas)
            self.detached_window = DetachedPlotWindow(self)
            self.detached_window.show()
            self.detach_btn.setText("Attach Plot")

    def create_pwm_slider(self, label, callback, min_val=0, max_val=1023):
        group = QGroupBox(label)
        layout = QVBoxLayout()
        
        # Add value label
        value_label = QLabel(f"{(min_val + max_val) // 2}")
        value_label.setAlignment(Qt.AlignCenter)
        value_label.setStyleSheet("font-weight: bold; color: #2b2d42;")
        
        slider = QSlider(Qt.Horizontal)
        slider.setMinimum(min_val)
        slider.setMaximum(max_val)
        slider.setValue((min_val + max_val) // 2)
        slider.setMinimumHeight(35)
        
        def on_value_change(val):
            value_label.setText(str(val))
            callback(val)
        
        slider.valueChanged.connect(on_value_change)
        
        layout.addWidget(value_label)
        layout.addWidget(slider)
        group.setLayout(layout)
        
        # Store slider and label for gamepad updates
        if label == "Shared Motor PWM":
            self.shared_pwm_label = value_label
            self.shared_pwm_slider_ref = slider
        elif label == "Wrist Servo (0-180°)":
            self.servo_label = value_label
            self.servo_slider_ref = slider
            
        return group

    def create_motor_control(self, name, motor_type):
        group = QGroupBox(name)
        layout = QHBoxLayout()
        
        # Forward button
        fwd_btn = QPushButton("FWD")
        fwd_btn.setCheckable(True)
        fwd_btn.setMinimumHeight(40)
        fwd_btn.clicked.connect(lambda: self.set_motor_state(motor_type, 1))
        
        # Backward button
        bwd_btn = QPushButton("BWD")
        bwd_btn.setCheckable(True)
        bwd_btn.setMinimumHeight(40)
        bwd_btn.clicked.connect(lambda: self.set_motor_state(motor_type, 2))
        
        # Stop button
        stop_btn = QPushButton("STOP")
        stop_btn.setCheckable(True)
        stop_btn.setMinimumHeight(40)
        stop_btn.clicked.connect(lambda: self.set_motor_state(motor_type, 0))
        
        # Create button group for exclusive selection
        btn_group = QButtonGroup(group)
        btn_group.addButton(fwd_btn, 1)
        btn_group.addButton(bwd_btn, 2)
        btn_group.addButton(stop_btn, 0)
        btn_group.setExclusive(True)
        stop_btn.setChecked(True)  # Default to stop
        
        layout.addWidget(fwd_btn)
        layout.addWidget(bwd_btn)
        layout.addWidget(stop_btn)
        group.setLayout(layout)
        
        # Store buttons for gamepad updates
        if motor_type == 'base':
            self.base_fwd_btn = fwd_btn
            self.base_bwd_btn = bwd_btn
            self.base_stop_btn = stop_btn
        elif motor_type == 'shoulder':
            self.shoulder_fwd_btn = fwd_btn
            self.shoulder_bwd_btn = bwd_btn
            self.shoulder_stop_btn = stop_btn
        elif motor_type == 'elbow':
            self.elbow_fwd_btn = fwd_btn
            self.elbow_bwd_btn = bwd_btn
            self.elbow_stop_btn = stop_btn
            
        return group

    def create_gripper_roller_control(self, name, control_type):
        group = QGroupBox(name)
        layout = QHBoxLayout()
        
        # Open button
        open_btn = QPushButton("Open")
        open_btn.setCheckable(True)
        open_btn.setMinimumHeight(40)
        open_btn.clicked.connect(lambda: self.set_gripper_roller_state(control_type, 2))
        
        # Close button
        close_btn = QPushButton("Close")
        close_btn.setCheckable(True)
        close_btn.setMinimumHeight(40)
        close_btn.clicked.connect(lambda: self.set_gripper_roller_state(control_type, 1))
        
        # Stop button
        stop_btn = QPushButton("Stop")
        stop_btn.setCheckable(True)
        stop_btn.setMinimumHeight(40)
        stop_btn.clicked.connect(lambda: self.set_gripper_roller_state(control_type, 0))
        
        # Create button group for exclusive selection
        btn_group = QButtonGroup(group)
        btn_group.addButton(open_btn, 2)
        btn_group.addButton(close_btn, 1)
        btn_group.addButton(stop_btn, 0)
        btn_group.setExclusive(True)
        stop_btn.setChecked(True)  # Default to stop
        
        layout.addWidget(open_btn)
        layout.addWidget(close_btn)
        layout.addWidget(stop_btn)
        group.setLayout(layout)
        
        # Store buttons for gamepad updates
        if control_type == 'gripper':
            self.gripper_open_btn = open_btn
            self.gripper_close_btn = close_btn
            self.gripper_stop_btn = stop_btn
        elif control_type == 'roller':
            self.roller_open_btn = open_btn
            self.roller_close_btn = close_btn
            self.roller_stop_btn = stop_btn
            
        return group

    def set_motor_state(self, motor_type, state):
        if motor_type == 'base':
            self.base_state = state
        elif motor_type == 'shoulder':
            self.shoulder_state = state
        elif motor_type == 'elbow':
            self.elbow_state = state

    def set_gripper_roller_state(self, control_type, state):
        if control_type == 'gripper':
            self.gripper_state = state
        elif control_type == 'roller':
            self.roller_state = state

    def get_direction_and_value(self, state):
        if state == 1:  # Forward
            return [1, self.shared_pwm]
        elif state == 2:  # Backward
            return [0, self.shared_pwm]
        else:  # Stop
            return [0, 0]

    def get_current_values(self):
        return [
            self.gripper_state,
            self.roller_state,
            self.servo_angle,
            self.get_direction_and_value(self.elbow_state),
            self.get_direction_and_value(self.shoulder_state),
            self.get_direction_and_value(self.base_state),
        ]

    def reset_all(self):
        # Reset motor states
        self.base_state = 0
        self.shoulder_state = 0
        self.elbow_state = 0
        
        # Reset gripper and roller states
        self.gripper_state = 0
        self.roller_state = 0
        
        # Reset servo angle
        self.servo_angle = 90
        
        # Reset PWM values
        self.shared_pwm = 0
        
        # Reset UI components
        for widget in self.findChildren(QSlider):
            if widget.minimum() == 0 and widget.maximum() == 180:  # Servo slider
                widget.setValue(90)
            else:  # PWM sliders
                widget.setValue(0)
        
        # Reset button groups
        for btn_group in self.findChildren(QButtonGroup):
            btn_group.setExclusive(False)
            for button in btn_group.buttons():
                button.setChecked(False)
            btn_group.setExclusive(True)
            
            # Set stop buttons to checked
            for button in btn_group.buttons():
                if btn_group.id(button) == 0:
                    button.setChecked(True)

    def update_output(self):
        values = self.get_current_values()
        
        # Update gamepad status
        if self.joystick:
            self.gamepad_status.setText("🟢 Gamepad connected")
            self.gamepad_status.setStyleSheet("font-weight: bold; padding: 5px; color: green;")
        else:
            self.gamepad_status.setText("🔴 No gamepad")
            self.gamepad_status.setStyleSheet("font-weight: bold; padding: 5px; color: red;")
        
        # Only send and update if values changed
        if values != self.last_values:
            logger.info(f"ARM Values: {values}")
            
            # Send via ROS2
            self.send_ros_message(values)
            
            # Update plot
            self.update_plot(values)
            self.last_values = values

    def update_plot(self, values):
        _, _, wrist, elbow, shoulder, base = values
        base_angle = (base[0]*2 - 1) * (base[1] / 1023) * 90
        shoulder_angle = (shoulder[0]*2 - 1) * (shoulder[1] / 1023) * 90
        elbow_angle = (elbow[0]*2 - 1) * (elbow[1] / 1023) * 90

        x0, y0, z0 = 0, 0, 0
        x1 = L1 * np.cos(np.radians(base_angle)) * np.cos(np.radians(shoulder_angle))
        y1 = L1 * np.sin(np.radians(base_angle)) * np.cos(np.radians(shoulder_angle))
        z1 = L1 * np.sin(np.radians(shoulder_angle))

        x2 = x1 + L2 * np.cos(np.radians(base_angle)) * np.cos(np.radians(shoulder_angle + elbow_angle))
        y2 = y1 + L2 * np.sin(np.radians(base_angle)) * np.cos(np.radians(shoulder_angle + elbow_angle))
        z2 = z1 + L2 * np.sin(np.radians(shoulder_angle + elbow_angle))

        x3 = x2 + L3 * np.cos(np.radians(base_angle)) * np.cos(np.radians(wrist))
        y3 = y2 + L3 * np.sin(np.radians(base_angle)) * np.cos(np.radians(wrist))
        z3 = z2 + L3 * np.sin(np.radians(wrist))

        self.ax.cla()
        self.ax.plot([x0, x1, x2, x3], [y0, y1, y2, y3], [z0, z1, z2, z3], 
                    color='#3f72af', marker='o', linewidth=3, markersize=8)
        
        # Add joint labels
        self.ax.text(x0, y0, z0, 'Base', fontsize=8)
        self.ax.text(x1, y1, z1, 'Shoulder', fontsize=8)
        self.ax.text(x2, y2, z2, 'Elbow', fontsize=8)
        self.ax.text(x3, y3, z3, 'Wrist', fontsize=8)
        
        # Set limits and labels
        self.ax.set_xlim(-30, 30)
        self.ax.set_ylim(-30, 30)
        self.ax.set_zlim(0, 50)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_title("3D Arm Position")
        self.ax.grid(True)
        self.canvas.draw()

    def update_gamepad(self):
        """Process gamepad inputs"""
        if not self.joystick:
            # Try to reconnect if gamepad wasn't detected at startup
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                logger.info(f"Gamepad connected: {self.joystick.get_name()}")
            return
            
        # Process pygame events
        for event in pygame.event.get():
            pass  # We're using state polling instead of events
        
        try:
            # Get current button states
            buttons = [self.joystick.get_button(i) for i in range(self.joystick.get_numbuttons())]
            hats = self.joystick.get_numhats()
            hat_state = (0, 0)
            if hats > 0:
                hat_state = self.joystick.get_hat(0)  # D-pad state
            
            # Get analog stick values
            axis_x = self.joystick.get_axis(0)  # Left stick horizontal
            axis_y = self.joystick.get_axis(1)  # Left stick vertical
            
            # Xbox controller mappings for your controller:
            # Buttons: [Y, B, A, X, LB, RB, LT, RT, BACK, START, L, R]
            # Indices:  0  1  2  3  4   5   6   7    8      9     10 11
            
            # Continuous adjustment for triggers
            current_time = pygame.time.get_ticks()
            time_since_last = current_time - self.last_gamepad_update
            if time_since_last < 50:  # Only update every 50ms for smooth control
                return
            
            self.last_gamepad_update = current_time
            
            # Shared PWM control with RT/LT
            if buttons[7]:  # RT (button 7) - increase shared PWM
                new_pwm = min(1023, self.shared_pwm + 10)
                if new_pwm != self.shared_pwm:
                    self.shared_pwm = new_pwm
                    self.shared_pwm_slider_ref.setValue(self.shared_pwm)
                    self.shared_pwm_label.setText(str(self.shared_pwm))
                    
            if buttons[6]:  # LT (button 6) - decrease shared PWM
                new_pwm = max(0, self.shared_pwm - 10)
                if new_pwm != self.shared_pwm:
                    self.shared_pwm = new_pwm
                    self.shared_pwm_slider_ref.setValue(self.shared_pwm)
                    self.shared_pwm_label.setText(str(self.shared_pwm))
            
            # Wrist servo control with RB/LB
            if buttons[5]:  # RB (button 5) - increase wrist servo
                new_angle = min(180, self.servo_angle + 1)
                if new_angle != self.servo_angle:
                    self.servo_angle = new_angle
                    self.servo_slider_ref.setValue(self.servo_angle)
                    self.servo_label.setText(str(self.servo_angle))
                    
            if buttons[4]:  # LB (button 4) - decrease wrist servo
                new_angle = max(0, self.servo_angle - 1)
                if new_angle != self.servo_angle:
                    self.servo_angle = new_angle
                    self.servo_slider_ref.setValue(self.servo_angle)
                    self.servo_label.setText(str(self.servo_angle))
            
            # BASE MOTOR control with left stick horizontal
            if abs(axis_x) > self.deadzone:
                if axis_x < 0:  # Left stick left = base forward
                    self.set_motor_state('base', 1)
                    self.base_fwd_btn.setChecked(True)
                else:  # Left stick right = base backward
                    self.set_motor_state('base', 2)
                    self.base_bwd_btn.setChecked(True)
            else:
                self.set_motor_state('base', 0)
                self.base_stop_btn.setChecked(True)
            
            # ELBOW MOTOR control with left stick vertical
            if abs(axis_y) > self.deadzone:
                if axis_y < 0:  # Left stick up = elbow up (forward)
                    self.set_motor_state('elbow', 1)
                    self.elbow_fwd_btn.setChecked(True)
                else:  # Left stick down = elbow down (backward)
                    self.set_motor_state('elbow', 2)
                    self.elbow_bwd_btn.setChecked(True)
            else:
                self.set_motor_state('elbow', 0)
                self.elbow_stop_btn.setChecked(True)
            
            # SHOULDER MOTOR control with buttons
            if buttons[2]:  # A button (index 2) - shoulder down (backward)
                self.set_motor_state('shoulder', 2)
                self.shoulder_bwd_btn.setChecked(True)
            elif buttons[0]:  # Y button (index 0) - shoulder up (forward)
                self.set_motor_state('shoulder', 1)
                self.shoulder_fwd_btn.setChecked(True)
            else:
                self.set_motor_state('shoulder', 0)
                self.shoulder_stop_btn.setChecked(True)
            
            # Gripper controls
            if hat_state[1] == 1:  # D-pad up
                self.set_gripper_roller_state('gripper', 2)  # Open
                self.gripper_open_btn.setChecked(True)
            elif hat_state[1] == -1:  # D-pad down
                self.set_gripper_roller_state('gripper', 1)  # Close
                self.gripper_close_btn.setChecked(True)
                
            if buttons[10] and not self.prev_buttons[10]:  # L button (index 10)
                self.set_gripper_roller_state('gripper', 0)  # Stop
                self.gripper_stop_btn.setChecked(True)
            
            # Roller controls
            if hat_state[0] == 1:  # D-pad right
                self.set_gripper_roller_state('roller', 2)  # Open
                self.roller_open_btn.setChecked(True)
            elif hat_state[0] == -1:  # D-pad left
                self.set_gripper_roller_state('roller', 1)  # Close
                self.roller_close_btn.setChecked(True)
                
            if buttons[11] and not self.prev_buttons[11]:  # R button (index 11)
                self.set_gripper_roller_state('roller', 0)  # Stop
                self.roller_stop_btn.setChecked(True)
            
            # Reset button (START)
            if buttons[9] and not self.prev_buttons[9]:
                self.reset_all()
            
            # Save current button states for next comparison
            self.prev_buttons = buttons
            
        except Exception as e:
            logger.error(f"Gamepad error: {e}")
            self.joystick = None

    def closeEvent(self, event):
        """Clean shutdown when GUI is closed"""
        logger.info("Shutting down ARM Control GUI...")
        
        # Clean up ROS2
        if rclpy.ok():
            rclpy.shutdown()
        
        # Clean up pygame
        pygame.quit()
        
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = ArmControlGUI()
    window.show()
    sys.exit(app.exec_())
