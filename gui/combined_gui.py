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

BACK : Switch between ARM and WHEEL control profile
(wheel control is same as before)

WHEEL CONTROLS:
Right Stick Left to Right: Speed control
Left Stick: Steer
START: Reset/Stop

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
from geometry_msgs.msg import Twist

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QPushButton, QSlider, QGroupBox, QGridLayout, QFrame,
    QButtonGroup, QSizePolicy
)
from PyQt5.QtCore import Qt, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Arrow, Circle

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

class WheelPublisherNode(Node):
    def __init__(self):
        super().__init__('wheel_publisher')
        self.wheel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        logger.info("Wheel Publisher Node initialized")
        self.active = True

    def wheel_drive(self, x: float, z: float, speed: float):
        if not self.active:
            return
            
        try:
            msg = Twist()
            msg.linear.x = x * speed
            msg.angular.z = z * speed
            self.wheel_pub.publish(msg)
            self.get_logger().info(f"WHEEL: x: {x:.2f} | z: {z:.2f} | speed: {speed:.2f}")
        except Exception as e:
            logger.error(f"Publishing error: {e}")

    def shutdown(self):
        self.active = False
        self.destroy_node()

class ArmControlWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.setup_ui()
        self.initialize_state()

    def setup_ui(self):
        layout = QVBoxLayout(self)
        layout.setAlignment(Qt.AlignTop)
        
        # Add profile indicator
        self.profile_label = QLabel("Arm Control (Active)")
        self.profile_label.setStyleSheet("font-weight: bold; padding: 5px; background-color: #4CAF50; color: white;")
        layout.addWidget(self.profile_label)
        
        # Shared PWM slider for base, shoulder, and elbow motors
        self.shared_pwm_slider = self.create_pwm_slider("Shared Motor PWM", lambda val: setattr(self, 'shared_pwm', val), 0, 1023)
        layout.addWidget(self.shared_pwm_slider)
        
        # Motor control sections
        layout.addWidget(self.create_motor_control("Base Motor", 'base'))
        layout.addWidget(self.create_motor_control("Shoulder Motor", 'shoulder'))
        layout.addWidget(self.create_motor_control("Elbow Motor", 'elbow'))
        
        # Servo control
        self.servo_slider = self.create_pwm_slider("Wrist Servo (0-180°)", lambda val: setattr(self, 'servo_angle', val), 0, 180)
        layout.addWidget(self.servo_slider)
        
        # Gripper and Roller controls
        layout.addWidget(self.create_gripper_roller_control("Gripper", 'gripper'))
        layout.addWidget(self.create_gripper_roller_control("Roller", 'roller'))

        btn_layout = QHBoxLayout()
        reset_btn = QPushButton("Reset")
        reset_btn.setMinimumHeight(40)
        reset_btn.clicked.connect(self.reset_all)
        btn_layout.addWidget(reset_btn)
        layout.addLayout(btn_layout)
        
        # Set styles
        self.setStyleSheet("""
            QWidget { background-color: #f4f4f4; font-family: 'Segoe UI', Arial; font-size: 12pt; }
            QGroupBox { border: 1px solid #cccccc; border-radius: 8px; margin-top: 1.5ex; padding: 10px; }
            QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
            QPushButton { background-color: #2b2d42; color: white; border-radius: 8px; padding: 8px; font-weight: bold; }
            QPushButton:hover { background-color: #1f2235; }
            QPushButton:checked { background-color: #4a4d6d; }
            QSlider::groove:horizontal { background: #cccccc; height: 10px; border-radius: 5px; }
            QSlider::handle:horizontal { background: #2b2d42; width: 30px; height: 30px; border-radius: 15px; margin: -10px 0; }
        """)
        
    def initialize_state(self):
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

        # Gamepad button state tracking
        self.prev_buttons = [False] * 15  # Track previous button states
        self.hat_state = (0, 0)  # Track D-pad state
        self.last_gamepad_update = pygame.time.get_ticks()
        
        # Gamepad axis deadzone
        self.deadzone = 0.2
        
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

        self.parent.arm_plot.ax.cla()
        self.parent.arm_plot.ax.plot([x0, x1, x2, x3], [y0, y1, y2, y3], [z0, z1, z2, z3], 
                    color='#3f72af', marker='o', linewidth=3, markersize=8)
        
        # Add joint labels
        self.parent.arm_plot.ax.text(x0, y0, z0, 'Base', fontsize=8)
        self.parent.arm_plot.ax.text(x1, y1, z1, 'Shoulder', fontsize=8)
        self.parent.arm_plot.ax.text(x2, y2, z2, 'Elbow', fontsize=8)
        self.parent.arm_plot.ax.text(x3, y3, z3, 'Wrist', fontsize=8)
        
        # Set limits and labels
        self.parent.arm_plot.ax.set_xlim(-30, 30)
        self.parent.arm_plot.ax.set_ylim(-30, 30)
        self.parent.arm_plot.ax.set_zlim(0, 50)
        self.parent.arm_plot.ax.set_xlabel("X")
        self.parent.arm_plot.ax.set_ylabel("Y")
        self.parent.arm_plot.ax.set_zlabel("Z")
        self.parent.arm_plot.ax.set_title("Arm Position")
        self.parent.arm_plot.ax.grid(True)
        self.parent.arm_plot.canvas.draw()

    def update_gamepad(self, joystick):
        """Process gamepad inputs for arm control"""
        if not joystick:
            return
            
        # Process pygame events
        for event in pygame.event.get():
            pass  # We're using state polling instead of events
        
        try:
            # Get current button states
            buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]
            hats = joystick.get_numhats()
            hat_state = (0, 0)
            if hats > 0:
                hat_state = joystick.get_hat(0)  # D-pad state
            
            # Get analog stick values
            axis_x = joystick.get_axis(0)  # Left stick horizontal
            axis_y = joystick.get_axis(1)  # Left stick vertical
            
            # Xbox controller mappings:
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
            
            # SHOULDER MOTOR control with D-pad up/down
            if hat_state[1] == 1:  # D-pad up
                self.set_motor_state('shoulder', 1)  # Forward (up)
                self.shoulder_fwd_btn.setChecked(True)
            elif hat_state[1] == -1:  # D-pad down
                self.set_motor_state('shoulder', 2)  # Backward (down)
                self.shoulder_bwd_btn.setChecked(True)
            else:
                self.set_motor_state('shoulder', 0)
                self.shoulder_stop_btn.setChecked(True)
            
            # Gripper controls with D-pad left/right
            if hat_state[0] == 1:  # D-pad right
                self.set_gripper_roller_state('gripper', 2)  # Open
                self.gripper_open_btn.setChecked(True)
            elif hat_state[0] == -1:  # D-pad left
                self.set_gripper_roller_state('gripper', 1)  # Close
                self.gripper_close_btn.setChecked(True)
                
            if buttons[10] and not self.prev_buttons[10]:  # L button (index 10)
                self.set_gripper_roller_state('gripper', 0)  # Stop
                self.gripper_stop_btn.setChecked(True)
            
            # Roller controls with X, B, A buttons
            if buttons[3]:  # X button (index 3) - roller open
                self.set_gripper_roller_state('roller', 2)  # Open
                self.roller_open_btn.setChecked(True)
            elif buttons[1]:  # B button (index 1) - roller close
                self.set_gripper_roller_state('roller', 1)  # Close
                self.roller_close_btn.setChecked(True)
            elif buttons[2]:  # A button (index 2) - roller stop
                self.set_gripper_roller_state('roller', 0)  # Stop
                self.roller_stop_btn.setChecked(True)
                
                self.set_gripper_roller_state('gripper', 0)  # Stop
                self.gripper_stop_btn.setChecked(True)
            
            # Save current button states for next comparison
            self.prev_buttons = buttons
            
        except Exception as e:
            logger.error(f"Gamepad error: {e}")

class WheelControlWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.parent = parent
        self.shutting_down = False
        self.setup_ui()
        self.initialize_state()
        
    def setup_ui(self):
        layout = QVBoxLayout(self)
        
        # Add profile indicator
        self.profile_label = QLabel("Wheel Control")
        self.profile_label.setStyleSheet("font-weight: bold; padding: 5px; background-color: #f0f0f0;")
        layout.addWidget(self.profile_label)
        
        # Speed control
        speed_group = QGroupBox("Speed Control")
        speed_layout = QVBoxLayout()
        
        self.speed_label = QLabel("0%")
        self.speed_label.setAlignment(Qt.AlignCenter)
        self.speed_label.setStyleSheet("font-weight: bold; color: #2b2d42;")
        
        self.speed_slider = QSlider(Qt.Horizontal)
        self.speed_slider.setRange(0, 100)
        self.speed_slider.setValue(0)
        self.speed_slider.setMinimumHeight(35)
        self.speed_slider.valueChanged.connect(self.update_speed)
        
        # Style the slider
        self.speed_slider.setStyleSheet("""
            QSlider::groove:horizontal {
                background: #cccccc;
                height: 10px;
                border-radius: 5px;
            }
            QSlider::handle:horizontal {
                background: #2b2d42;
                width: 30px;
                height: 30px;
                margin: -10px 0;
                border-radius: 15px;
            }
        """)
        
        speed_layout.addWidget(self.speed_label)
        speed_layout.addWidget(self.speed_slider)
        speed_group.setLayout(speed_layout)
        layout.addWidget(speed_group)

        # Direction buttons
        btn_group = QGroupBox("Direction Control")
        grid = QGridLayout()
        grid.setSpacing(15)
        grid.setContentsMargins(20, 20, 20, 20)
        
        # Create buttons
        self.btn_forward = QPushButton("↑ FORWARD")
        self.btn_backward = QPushButton("↓ BACKWARD")
        self.btn_left = QPushButton("← LEFT")
        self.btn_right = QPushButton("→ RIGHT")
        self.btn_stop = QPushButton("STOP")
        
        # Set button styles
        for btn in [self.btn_forward, self.btn_backward, self.btn_left, self.btn_right]:
            btn.setMinimumHeight(60)
            btn.setStyleSheet("""
                QPushButton {
                    background-color: #2b2d42; 
                    color: white; 
                    border-radius: 10px; 
                    padding: 15px; 
                    font-weight: bold;
                    font-size: 16px;
                }
                QPushButton:pressed {
                    background-color: #4a4d6d;
                }
            """)
        
        self.btn_stop.setMinimumHeight(60)
        self.btn_stop.setStyleSheet("""
            QPushButton {
                background-color: #e74c3c; 
                color: white; 
                border-radius: 10px; 
                padding: 15px; 
                font-weight: bold;
                font-size: 16px;
            }
            QPushButton:pressed {
                background-color: #c0392b;
            }
        """)
        
        # Connect signals with press/release for toggle behavior
        self.btn_forward.pressed.connect(lambda: self.set_direction_button('forward', True))
        self.btn_forward.released.connect(lambda: self.set_direction_button('forward', False))
        self.btn_backward.pressed.connect(lambda: self.set_direction_button('backward', True))
        self.btn_backward.released.connect(lambda: self.set_direction_button('backward', False))
        self.btn_left.pressed.connect(lambda: self.set_direction_button('left', True))
        self.btn_left.released.connect(lambda: self.set_direction_button('left', False))
        self.btn_right.pressed.connect(lambda: self.set_direction_button('right', True))
        self.btn_right.released.connect(lambda: self.set_direction_button('right', False))
        self.btn_stop.clicked.connect(self.stop_all)
        
        # Layout
        grid.addWidget(self.btn_forward, 0, 1)
        grid.addWidget(self.btn_left, 1, 0)
        grid.addWidget(self.btn_stop, 1, 1)
        grid.addWidget(self.btn_right, 1, 2)
        grid.addWidget(self.btn_backward, 2, 1)
        
        btn_group.setLayout(grid)
        btn_group.setStyleSheet("""
            QGroupBox {
                border: 1px solid #cccccc;
                border-radius: 8px;
                margin-top: 1.5ex;
                padding: 10px;
            }
            QGroupBox::title {
                subcontrol-origin: margin;
                left: 10px;
                padding: 0 5px;
            }
        """)
        layout.addWidget(btn_group)
        
    def initialize_state(self):
        # Control variables
        self.speed = 0.0
        self.x_dir = 0.0  # Forward/backward direction
        self.z_dir = 0.0  # Left/right steering
        self.button_states = {
            'forward': False,
            'backward': False,
            'left': False,
            'right': False
        }
        
        # Track last published values
        self.last_x = 0.0
        self.last_z = 0.0
        self.last_speed = 0.0
        
        # Deadzone for change detection
        self.deadzone = 0.01  # 1% change threshold

    def set_direction_button(self, direction, pressed):
        """Set direction based on button press/release"""
        self.button_states[direction] = pressed
        self.update_direction_from_buttons()
        self.parent.wheel_plot.update_plot(self.x_dir, self.z_dir)

    def update_direction_from_buttons(self):
        """Calculate direction from button states"""
        x = 0.0
        z = 0.0
        
        if self.button_states['forward']:
            x += 1.0
        if self.button_states['backward']:
            x -= 1.0
        if self.button_states['left']:
            z += 1.0
        if self.button_states['right']:
            z -= 1.0
        
        # Normalize if both directions pressed
        if x != 0 and z != 0:
            magnitude = np.sqrt(x**2 + z**2)
            x /= magnitude
            z /= magnitude
        
        self.x_dir = x
        self.z_dir = z

    def update_speed(self, value):
        """Handle speed slider changes"""
        self.speed = value / 100.0
        self.speed_label.setText(f"{value}%")

    def stop_all(self):
        """Stop all movement"""
        self.speed_slider.setValue(0)
        self.x_dir = 0.0
        self.z_dir = 0.0
        self.parent.wheel_plot.update_plot(0, 0)
        
        # Reset all button states
        for direction in self.button_states:
            self.button_states[direction] = False
        
        # Visually unpress all buttons
        self.btn_forward.setDown(False)
        self.btn_backward.setDown(False)
        self.btn_left.setDown(False)
        self.btn_right.setDown(False)
        
        # Force publish stop command
        if self.parent.ros_wheel_node and self.parent.ros_wheel_node.active:
            try:
                self.parent.ros_wheel_node.wheel_drive(0.0, 0.0, 0.0)
                # Update last published values
                self.last_x = 0.0
                self.last_z = 0.0
                self.last_speed = 0.0
            except Exception as e:
                logger.error(f"Error publishing stop command: {e}")

    def update_gamepad(self, joystick):
        """Process gamepad inputs for wheel control"""
        if not joystick:
            return
            
        try:
            # Process events
            pygame.event.pump()
            
            # Get axis values safely
            num_axes = joystick.get_numaxes()
            left_x = 0.0
            left_y = 0.0
            right_x = 0.0  # Right stick X for speed control

            if num_axes > 3:
                right_x = joystick.get_axis(3)
            if num_axes > 2:
                left_x = joystick.get_axis(2)
            
            if num_axes > 0:
                left_x = joystick.get_axis(0)
            if num_axes > 1:
                left_y = joystick.get_axis(1)
            
            current_slider_val = self.speed_slider.value()
            change = int(right_x * 5)  # Adjust step size as needed
            new_speed_val = max(0, min(100, current_slider_val + change))
            self.speed_slider.setValue(new_speed_val)
            self.speed = new_speed_val / 100.0
            
            # Calculate direction from left stick
            deadzone = 0.2
            self.x_dir = -left_y if abs(left_y) > deadzone else 0.0
            self.z_dir = left_x if abs(left_x) > deadzone else 0.0
            
            # Normalize direction vector
            magnitude = np.sqrt(self.x_dir**2 + self.z_dir**2)
            if magnitude > 1.0:
                self.x_dir /= magnitude
                self.z_dir /= magnitude
                
            # Update UI buttons based on direction
            self.btn_forward.setDown(self.x_dir > 0.1)
            self.btn_backward.setDown(self.x_dir < -0.1)
            self.btn_left.setDown(self.z_dir > 0.1)
            self.btn_right.setDown(self.z_dir < -0.1)
            
            # Update plot
            self.parent.wheel_plot.update_plot(self.x_dir, self.z_dir)
            
        except Exception as e:
            logger.error(f"Gamepad error: {e}")

class ArmPlotWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_plot()
        
    def setup_plot(self):
        layout = QVBoxLayout(self)
        
        self.figure = Figure(figsize=(5, 5))
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111, projection='3d')
        self.ax.set_xlim(-30, 30)
        self.ax.set_ylim(-30, 30)
        self.ax.set_zlim(0, 50)
        self.ax.set_xlabel("X")
        self.ax.set_ylabel("Y")
        self.ax.set_zlabel("Z")
        self.ax.set_title("Arm Position")
        self.ax.grid(True)
        
        layout.addWidget(self.canvas)
        self.setLayout(layout)

class WheelPlotWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_plot()
        
    def setup_plot(self):
        layout = QVBoxLayout(self)
        
        self.figure = Figure(figsize=(5, 5))
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlim(-1.5, 1.5)
        self.ax.set_ylim(-1.5, 1.5)
        self.ax.grid(True, linestyle='--', alpha=0.7)
        self.ax.set_title("Movement Direction", fontsize=10, fontweight='bold')
        self.ax.set_xlabel("Left/Right", fontsize=12)
        self.ax.set_ylabel("Forward/Backward", fontsize=8)
        
        # Add center point
        self.center_circle = Circle((0, 0), 0.05, color='#3498db', zorder=5)
        self.ax.add_patch(self.center_circle)
        
        # Initial arrow
        self.direction_arrow = None
        
        # Add axis labels
        self.ax.text(1.4, 0.05, "→", fontsize=16, ha='center', va='center')
        self.ax.text(0.05, 1.4, "↑", fontsize=16, ha='center', va='center')
        self.ax.text(0, -1.4, "BACKWARD", ha='center', va='center', fontsize=10)
        self.ax.text(0, 1.4, "FORWARD", ha='center', va='center', fontsize=10)
        self.ax.text(-1.4, 0, "LEFT", ha='center', va='center', fontsize=10, rotation=90)
        self.ax.text(1.4, 0, "RIGHT", ha='center', va='center', fontsize=10, rotation=-90)
        
        layout.addWidget(self.canvas)
        self.setLayout(layout)
        
    def update_plot(self, x_dir, z_dir):
        """Update the direction visualization"""
        # Clear previous arrow if it exists
        if self.direction_arrow:
            try:
                self.direction_arrow.remove()
            except ValueError:
                pass
            self.direction_arrow = None
        
        # Create new arrow if needed
        if x_dir != 0 or z_dir != 0:
            # Arrow points: (x=left/right, y=forward/backward)
            dx = z_dir  # Left/Right
            dy = x_dir  # Forward/Backward
            
            # Create a new arrow
            self.direction_arrow = Arrow(0, 0, dx, dy, width=0.2, color='#e74c3c', zorder=4)
            self.ax.add_patch(self.direction_arrow)
            
            # Add magnitude indicator
            magnitude = np.sqrt(dx**2 + dy**2)
            if magnitude > 0.1:
                self.ax.text(dx/2, dy/2, f"{magnitude*100:.0f}%", 
                            fontsize=10, ha='center', va='center',
                            bbox=dict(facecolor='white', alpha=0.7, edgecolor='none'))
        
        self.canvas.draw()

class MarsRoverControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Team Interplanetar")
        self.setGeometry(100, 100, 1800, 900)
        self.shutting_down = False
        
        # Control profile management
        self.control_profile = 'arm'  # 'arm' or 'wheel'
        self.prev_start_button = False  # Track previous state of START button
        
        # Initialize pygame for gamepad support
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        self.init_gamepad()
        
        # ROS2 setup
        rclpy.init()
        self.setup_ros_nodes()
        
        # Create main layout
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.main_layout = QHBoxLayout(self.main_widget)
        self.main_layout.setSpacing(15)
        self.main_layout.setContentsMargins(15, 15, 15, 15)
        
        # Create sections
        self.setup_arm_section()
        self.setup_wheel_section()
        self.setup_plot_section()
        
        # Status bar
        self.status_bar = self.statusBar()
        self.status_bar.setStyleSheet("background-color: #f0f0f0;")
        self.update_status_bar()
        
        # Timers
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_output)
        self.timer.start(50)  # 20Hz update
        
        self.gamepad_timer = QTimer()
        self.gamepad_timer.timeout.connect(self.update_gamepad)
        self.gamepad_timer.start(100)  # 10Hz update
        
    def init_gamepad(self):
        """Initialize gamepad if available"""
        if pygame.joystick.get_count() > 0:
            try:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                logger.info(f"Gamepad connected: {self.joystick.get_name()}")
            except Exception as e:
                logger.error(f"Gamepad init error: {e}")
                self.joystick = None
        else:
            logger.warning("No gamepad detected")
            self.joystick = None
            
    def setup_ros_nodes(self):
        # Arm node
        self.ros_arm_node = ArmCommandPublisher()
        self.arm_thread = threading.Thread(target=self.arm_ros_spin, daemon=True)
        self.arm_thread.start()
        
        # Wheel node
        self.ros_wheel_node = WheelPublisherNode()
        self.wheel_thread = threading.Thread(target=self.wheel_ros_spin, daemon=True)
        self.wheel_thread.start()
        
    def arm_ros_spin(self):
        """Spin the ROS node in a separate thread"""
        rclpy.spin(self.ros_arm_node)
        
    def wheel_ros_spin(self):
        """Spin the wheel ROS node"""
        while rclpy.ok() and not self.shutting_down:
            rclpy.spin_once(self.ros_wheel_node, timeout_sec=0.1)
        
    def setup_arm_section(self):
        # Arm control section
        arm_frame = QFrame()
        arm_frame.setFrameShape(QFrame.StyledPanel)
        arm_layout = QVBoxLayout(arm_frame)
        
        self.arm_control = ArmControlWidget(self)
        arm_layout.addWidget(self.arm_control)
        
        self.main_layout.addWidget(arm_frame, 3)  # 30% width
        
    def setup_wheel_section(self):
        # Wheel control section
        wheel_frame = QFrame()
        wheel_frame.setFrameShape(QFrame.StyledPanel)
        wheel_layout = QVBoxLayout(wheel_frame)
        
        self.wheel_control = WheelControlWidget(self)
        wheel_layout.addWidget(self.wheel_control)
        
        self.main_layout.addWidget(wheel_frame, 3)  # 30% width
        
    def setup_plot_section(self):
        # Plots section
        plot_frame = QFrame()
        plot_frame.setFrameShape(QFrame.StyledPanel)
        plot_layout = QVBoxLayout(plot_frame)
        plot_layout.setSpacing(15)
        
        # Arm plot
        self.arm_plot = ArmPlotWidget(self)
        plot_layout.addWidget(self.arm_plot, 1)
        
        # Wheel plot
        self.wheel_plot = WheelPlotWidget(self)
        plot_layout.addWidget(self.wheel_plot, 1)
        
        self.main_layout.addWidget(plot_frame, 4)  # 40% width
        
    def update_status_bar(self):
        """Update status bar with current state"""
        ros_connected = rclpy.ok()
        gamepad_connected = self.joystick is not None
        
        status_text = "ROS: 🟢 Connected | " if ros_connected else "ROS: 🔴 Disconnected | "
        status_text += "Gamepad: 🟢 Connected | " if gamepad_connected else "Gamepad: 🔴 Not Connected | "
        status_text += f"Controller Profile: {'🟩 ARM' if self.control_profile == 'arm' else '🟢 WHEEL'}"
        
        self.status_bar.showMessage(status_text)
        
    def toggle_control_profile(self):
        """Switch between arm and wheel control profiles"""
        self.control_profile = 'wheel' if self.control_profile == 'arm' else 'arm'
        
        # Update UI indicators
        if self.control_profile == 'arm':
            self.arm_control.profile_label.setText("Arm Control (Active)")
            self.arm_control.profile_label.setStyleSheet(
                "font-weight: bold; padding: 5px; background-color: #4CAF50; color: white;"
            )
            self.wheel_control.profile_label.setText("Wheel Control")
            self.wheel_control.profile_label.setStyleSheet(
                "font-weight: bold; padding: 5px; background-color: #f0f0f0;"
            )
        else:
            self.arm_control.profile_label.setText("Arm Control")
            self.arm_control.profile_label.setStyleSheet(
                "font-weight: bold; padding: 5px; background-color: #f0f0f0;"
            )
            self.wheel_control.profile_label.setText("Wheel Control (Active)")
            self.wheel_control.profile_label.setStyleSheet(
                "font-weight: bold; padding: 5px; background-color: #4CAF50; color: white;"
            )
            
        self.update_status_bar()
        
    def reset_gui(self):
        """Reset both arm and wheel controls (kill switch)"""
        logger.info("Resetting GUI via kill switch")
        
        # Reset arm control
        self.arm_control.reset_all()
        
        # Reset wheel control
        self.wheel_control.stop_all()
        
        # Reset plots
        self.arm_plot.ax.cla()
        self.arm_plot.ax.set_xlim(-30, 30)
        self.arm_plot.ax.set_ylim(-30, 30)
        self.arm_plot.ax.set_zlim(0, 50)
        self.arm_plot.ax.set_xlabel("X")
        self.arm_plot.ax.set_ylabel("Y")
        self.arm_plot.ax.set_zlabel("Z")
        self.arm_plot.ax.set_title("Arm Position")
        self.arm_plot.ax.grid(True)
        self.arm_plot.canvas.draw()
        
        self.wheel_plot.update_plot(0, 0)
        
        # Send stop commands through ROS
        try:
            # Send arm reset command
            arm_values = self.arm_control.get_current_values()
            self.ros_arm_node.publish_command(arm_values)
            
            # Send wheel stop command
            self.ros_wheel_node.wheel_drive(0.0, 0.0, 0.0)
        except Exception as e:
            logger.error(f"Error sending reset commands: {e}")
        
    def update_gamepad(self):
        """Handle gamepad input and profile switching"""
        if not self.joystick:
            # Try to reconnect
            pygame.joystick.quit()
            pygame.joystick.init()
            if pygame.joystick.get_count() > 0:
                try:
                    self.joystick = pygame.joystick.Joystick(0)
                    self.joystick.init()
                    logger.info(f"Gamepad reconnected: {self.joystick.get_name()}")
                except Exception as e:
                    logger.error(f"Gamepad reconnect error: {e}")
                    self.joystick = None
                    
        # Process events
        pygame.event.pump()
        
        try:
            # Check for profile toggle button (BACK button - index 8)
            if self.joystick and self.joystick.get_button(8):
                self.toggle_control_profile()
                
            # Check for kill switch button (START button - index 9)
            if self.joystick:
                current_start = self.joystick.get_button(9)
                # Only trigger on button press (not release)
                if current_start and not self.prev_start_button:
                    self.reset_gui()
                self.prev_start_button = current_start
        except Exception as e:
            logger.error(f"Gamepad button error: {e}")
            
        # Route input to active control
        if self.joystick:
            if self.control_profile == 'arm':
                self.arm_control.update_gamepad(self.joystick)
            else:
                self.wheel_control.update_gamepad(self.joystick)
                
        self.update_status_bar()
        
    def update_output(self):
        """Update both control systems"""
        # Update arm plot if needed
        arm_values = self.arm_control.get_current_values()
        if arm_values != self.arm_control.last_values:
            try:
                self.ros_arm_node.publish_command(arm_values)
                self.arm_control.update_plot(arm_values)
                self.arm_control.last_values = arm_values
            except Exception as e:
                logger.error(f"Error publishing arm command: {e}")
                
        # Update wheel control
        if self.ros_wheel_node and self.ros_wheel_node.active:
            # Check if values have changed significantly
            x_changed = abs(self.wheel_control.x_dir - self.wheel_control.last_x) > self.wheel_control.deadzone
            z_changed = abs(self.wheel_control.z_dir - self.wheel_control.last_z) > self.wheel_control.deadzone
            speed_changed = abs(self.wheel_control.speed - self.wheel_control.last_speed) > self.wheel_control.deadzone
            
            # Publish only if there's a significant change
            if x_changed or z_changed or speed_changed:
                try:
                    self.ros_wheel_node.wheel_drive(
                        self.wheel_control.x_dir,
                        self.wheel_control.z_dir,
                        self.wheel_control.speed
                    )
                    # Update last published values
                    self.wheel_control.last_x = self.wheel_control.x_dir
                    self.wheel_control.last_z = self.wheel_control.z_dir
                    self.wheel_control.last_speed = self.wheel_control.speed
                except Exception as e:
                    logger.error(f"Error publishing to ROS: {e}")
        
    def closeEvent(self, event):
        """Cleanup on window close"""
        logger.info("Shutting down the GUI...")
        self.shutting_down = True
        
        # Stop timers first
        self.timer.stop()
        self.gamepad_timer.stop()
        
        # Clean up ROS
        if rclpy.ok():
            self.ros_arm_node.destroy_node()
            self.ros_wheel_node.shutdown()
            rclpy.shutdown()
            
        # Clean up pygame
        pygame.quit()
        
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = MarsRoverControlGUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
