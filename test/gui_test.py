import sys
import pygame
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QSlider
from PyQt5.QtCore import Qt, QTimer

class TriggerButtonSliderControl(QWidget):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("LT/RT Button Slider Control")
        self.resize(300, 150)

        # Slider setup
        self.slider = QSlider(Qt.Horizontal)
        self.slider.setMinimum(0)
        self.slider.setMaximum(100)
        self.slider.setValue(50)

        # Labels
        self.label = QLabel("Press RT to increase, LT to decrease")
        self.value_label = QLabel(f"Slider Value: {self.slider.value()}")

        # Layout
        layout = QVBoxLayout()
        layout.addWidget(self.label)
        layout.addWidget(self.slider)
        layout.addWidget(self.value_label)
        self.setLayout(layout)

        # Initialize Pygame Joystick
        pygame.init()
        pygame.joystick.init()
        self.joystick = None

        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.label.setText("Joystick connected.")
        else:
            self.label.setText("No joystick detected.")

        # Timer for polling joystick
        self.timer = QTimer()
        self.timer.timeout.connect(self.update_slider_from_buttons)
        self.timer.start(50)  # Check every 50 ms

    def update_slider_from_buttons(self):
        if not self.joystick:
            return

        pygame.event.pump()

        # Button indices (0-based)
        BUTTON_LT = 9
        BUTTON_RT = 10

        current = self.slider.value()
        step = 2

        if self.joystick.get_button(BUTTON_RT):
            current += step
        if self.joystick.get_button(BUTTON_LT):
            current -= step

        # Clamp the slider value
        current = max(self.slider.minimum(), min(self.slider.maximum(), current))
        self.slider.setValue(current)
        self.value_label.setText(f"Slider Value: {current}")

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = TriggerButtonSliderControl()
    window.show()
    sys.exit(app.exec_())
