import sys
import pygame
import numpy as np
import threading
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout, 
    QLabel, QPushButton, QSlider, QGroupBox, QGridLayout
)
from PyQt5.QtCore import Qt, QTimer
from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg as FigureCanvas
from matplotlib.figure import Figure
from matplotlib.patches import Arrow, Circle

# Configure logging
import logging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger(__name__)

class WheelPublisherNode(Node):
    def __init__(self):
        super().__init__('wheel_publisher')
        self.wheel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        logger.info("Wheel Publisher Node initialized")
        self.active = True  # Track if node is active

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

class WheelControlGUI(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Mars Rover Wheel Control")
        self.setGeometry(100, 100, 500, 800)
        self.shutting_down = False

        # Initialize pygame for gamepad support
        pygame.init()
        pygame.joystick.init()
        self.joystick = None
        self.init_gamepad()

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

        # ROS2 setup
        self.ros_node = None
        self.setup_ros_node()

        # GUI setup
        self.main_widget = QWidget()
        self.setCentralWidget(self.main_widget)
        self.main_layout = QVBoxLayout(self.main_widget)
        self.main_layout.setSpacing(15)
        self.main_layout.setContentsMargins(20, 20, 20, 20)
        
        self.init_controls()
        self.init_plot()

        # Timers
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_output)
        self.timer.start(50)  # 20Hz update
        
        self.gamepad_timer = QTimer()
        self.gamepad_timer.timeout.connect(self.update_gamepad)
        self.gamepad_timer.start(100)  # 10Hz update

        # Track last published values
        self.last_x = 0.0
        self.last_z = 0.0
        self.last_speed = 0.0
        
        # Deadzone for change detection
        self.deadzone = 0.01  # 1% change threshold

    def init_gamepad(self):
        """Initialize gamepad if available"""
        pygame.joystick.init()
        if pygame.joystick.get_count() > 0:
            try:
                self.joystick = pygame.joystick.Joystick(0)
                self.joystick.init()
                logger.info(f"Gamepad connected: {self.joystick.get_name()}")
                
                # Log gamepad details for debugging
                logger.info(f"Gamepad axes: {self.joystick.get_numaxes()}")
                logger.info(f"Gamepad buttons: {self.joystick.get_numbuttons()}")
            except Exception as e:
                logger.error(f"Gamepad init error: {e}")
                self.joystick = None
        else:
            logger.warning("No gamepad detected")
            self.joystick = None

    def setup_ros_node(self):
        """Setup ROS2 node in a separate thread"""
        def ros_thread_function():
            try:
                rclpy.init()
                self.ros_node = WheelPublisherNode()
                logger.info("ROS2 node initialized")
                
                # Spin until shutdown
                while rclpy.ok() and not self.shutting_down:
                    rclpy.spin_once(self.ros_node, timeout_sec=0.1)
                
                if self.ros_node:
                    self.ros_node.shutdown()
                rclpy.shutdown()
            except Exception as e:
                logger.error(f"ROS node error: {e}")

        self.ros_thread = threading.Thread(target=ros_thread_function, daemon=True)
        self.ros_thread.start()

    def init_controls(self):
        # Status indicators
        status_layout = QHBoxLayout()
        self.ros_status = QLabel("🔴 ROS Disconnected")
        self.ros_status.setStyleSheet("font-weight: bold; padding: 5px;")
        self.gamepad_status = QLabel("🔴 No Gamepad")
        self.gamepad_status.setStyleSheet("font-weight: bold; padding: 5px;")
        status_layout.addWidget(self.ros_status)
        status_layout.addWidget(self.gamepad_status)
        self.main_layout.addLayout(status_layout)

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
        self.main_layout.addWidget(speed_group)

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
        self.main_layout.addWidget(btn_group)

    def init_plot(self):
        """Initialize the direction visualization plot"""
        plot_group = QGroupBox("Direction Visualization")
        plot_layout = QVBoxLayout()
        
        self.figure = Figure(figsize=(8, 6))
        self.canvas = FigureCanvas(self.figure)
        self.ax = self.figure.add_subplot(111)
        self.ax.set_xlim(-1.5, 1.5)
        self.ax.set_ylim(-1.5, 1.5)
        self.ax.grid(True, linestyle='--', alpha=0.7)
        self.ax.set_title("Movement Direction", fontsize=10, fontweight='bold')
        self.ax.set_xlabel("Left/Right", fontsize=12)
        self.ax.set_ylabel("Forward/Backward", fontsize=8)
        
        # Add center point and direction arrow
        self.center_circle = Circle((0, 0), 0.05, color='#3498db', zorder=5)
        self.ax.add_patch(self.center_circle)
        
        # Initial arrow (will be updated)
        self.direction_arrow = None
        
        # Add axis labels
        self.ax.text(1.4, 0.05, "→", fontsize=16, ha='center', va='center')
        self.ax.text(0.05, 1.4, "↑", fontsize=16, ha='center', va='center')
        self.ax.text(0, -1.4, "BACKWARD", ha='center', va='center', fontsize=10)
        self.ax.text(0, 1.4, "FORWARD", ha='center', va='center', fontsize=10)
        self.ax.text(-1.4, 0, "LEFT", ha='center', va='center', fontsize=10, rotation=90)
        self.ax.text(1.4, 0, "RIGHT", ha='center', va='center', fontsize=10, rotation=-90)
        
        plot_layout.addWidget(self.canvas)
        plot_group.setLayout(plot_layout)
        plot_group.setStyleSheet("""
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
        self.main_layout.addWidget(plot_group)

    def set_direction_button(self, direction, pressed):
        """Set direction based on button press/release"""
        self.button_states[direction] = pressed
        self.update_direction_from_buttons()
        self.update_plot()

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
        self.update_plot()
        
        # Reset all button states
        for direction in self.button_states:
            self.button_states[direction] = False
        
        # Visually unpress all buttons
        self.btn_forward.setDown(False)
        self.btn_backward.setDown(False)
        self.btn_left.setDown(False)
        self.btn_right.setDown(False)

    def update_plot(self):
        """Update the direction visualization"""
        # Clear previous arrow if it exists
        if self.direction_arrow:
            try:
                self.direction_arrow.remove()
            except ValueError:
                # Arrow was already removed, ignore error
                pass
            self.direction_arrow = None
        
        # Create new arrow if needed
        if self.x_dir != 0 or self.z_dir != 0:
            # Arrow points: (x=left/right, y=forward/backward)
            dx = self.z_dir  # Left/Right
            dy = self.x_dir  # Forward/Backward
            
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

    def update_gamepad(self):
        """Process gamepad inputs"""
        if self.shutting_down:
            return
            
        # Try to reconnect if gamepad not detected
        if not self.joystick:
            try:
                pygame.joystick.quit()
                pygame.joystick.init()
                if pygame.joystick.get_count() > 0:
                    self.joystick = pygame.joystick.Joystick(0)
                    self.joystick.init()
                    logger.info(f"Gamepad reconnected: {self.joystick.get_name()}")
                    
                    # Log gamepad details for debugging
                    logger.info(f"Gamepad axes: {self.joystick.get_numaxes()}")
                    logger.info(f"Gamepad buttons: {self.joystick.get_numbuttons()}")
            except Exception as e:
                logger.error(f"Gamepad reconnect error: {e}")
                self.joystick = None
        
        if not self.joystick:
            return
            
        try:
            # Process events
            pygame.event.pump()
            
            # Get axis values safely
            num_axes = self.joystick.get_numaxes()
            left_x = 0.0
            left_y = 0.0
            # lt = 0.0
            # rt = 0.0
            num_axes = self.joystick.get_numaxes()
            right_x = 0.0  # Right stick X for speed control

            if num_axes > 3:
                right_x = self.joystick.get_axis(3)
            if num_axes > 2:
                left_x = self.joystick.get_axis(2)
            
            if num_axes > 0:
                left_x = self.joystick.get_axis(0)
            if num_axes > 1:
                left_y = self.joystick.get_axis(1)
            # if num_axes > 7:
            #     rt = (self.joystick.get_axis(7) + 1) / 2
            # if num_axes > 8:
            #     lt = (self.joystick.get_axis(8) + 1) / 2
            # else:
            #     # Try alternative axis for triggers if available
            #     if num_axes > 4:
            #         rt = (self.joystick.get_axis(4) + 1) / 2
            #     if num_axes > 3:
            #         lt = (self.joystick.get_axis(3) + 1) / 2
            
            # Calculate speed from triggers
            # speed_value = int((right_x) * 100)
            # self.speed_slider.setValue(max(0, min(100, abs(speed_value))))
            # self.speed = abs(speed_value) / 100.0

            current_slider_val = self.speed_slider.value()
            change = int(right_x * 5)  # tweak step size as needed
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
                
            # START button to stop
            if self.joystick.get_button(9):  # START button
                self.stop_all()
                
            # Update plot
            self.update_plot()
            
        except Exception as e:
            logger.error(f"Gamepad error: {e}")
            self.joystick = None

    def update_output(self):
        """Publish control values to ROS only when changed"""
        if self.shutting_down:
            return
            
        # Update status indicators
        ros_connected = self.ros_node is not None and self.ros_node.active
        gamepad_connected = self.joystick is not None
        
        self.ros_status.setText("🟢 ROS Connected" if ros_connected else "🔴 ROS Disconnected")
        self.ros_status.setStyleSheet(f"color: {'green' if ros_connected else 'red'}; font-weight: bold;")
        
        self.gamepad_status.setText("🟢 Gamepad Connected" if gamepad_connected else "🔴 No Gamepad")
        self.gamepad_status.setStyleSheet(f"color: {'green' if gamepad_connected else 'red'}; font-weight: bold;")
        
        # Check if values have changed significantly
        x_changed = abs(self.x_dir - self.last_x) > self.deadzone
        z_changed = abs(self.z_dir - self.last_z) > self.deadzone
        speed_changed = abs(self.speed - self.last_speed) > self.deadzone
        
        # Publish only if there's a significant change
        if ros_connected and (x_changed or z_changed or speed_changed):
            try:
                self.ros_node.wheel_drive(
                    self.x_dir,
                    self.z_dir,
                    self.speed
                )
                # Update last published values
                self.last_x = self.x_dir
                self.last_z = self.z_dir
                self.last_speed = self.speed
            except Exception as e:
                logger.error(f"Error publishing to ROS: {e}")

    def stop_all(self):
        """Stop all movement"""
        self.speed_slider.setValue(0)
        self.x_dir = 0.0
        self.z_dir = 0.0
        self.update_plot()
        
        # Reset all button states
        for direction in self.button_states:
            self.button_states[direction] = False
        
        # Visually unpress all buttons
        self.btn_forward.setDown(False)
        self.btn_backward.setDown(False)
        self.btn_left.setDown(False)
        self.btn_right.setDown(False)
        
        # Force publish stop command
        if self.ros_node and self.ros_node.active:
            try:
                self.ros_node.wheel_drive(0.0, 0.0, 0.0)
                # Update last published values
                self.last_x = 0.0
                self.last_z = 0.0
                self.last_speed = 0.0
            except Exception as e:
                logger.error(f"Error publishing stop command: {e}")

    def closeEvent(self, event):
        """Cleanup on window close"""
        logger.info("Shutting down Wheel Control GUI...")
        self.shutting_down = True
        
        # Stop timers first
        self.timer.stop()
        self.gamepad_timer.stop()
        
        # Shutdown ROS node
        if self.ros_node:
            self.ros_node.shutdown()
            
        # Clean up pygame
        pygame.quit()
        
        event.accept()

def main():
    app = QApplication(sys.argv)
    window = WheelControlGUI()
    window.show()
    sys.exit(app.exec_())

if __name__ == '__main__':
    main()
