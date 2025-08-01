import pygame

pygame.init()
pygame.joystick.init()

if pygame.joystick.get_count() == 0:
    print("No joystick detected.")
    exit()

joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Joystick Name: {joystick.get_name()}")
print(f"Number of Axes: {joystick.get_numaxes()}")
print(f"Number of Buttons: {joystick.get_numbuttons()}")

print("Move sticks and press triggers/buttons... (Press Ctrl+C to quit)")

try:
    while True:
        pygame.event.pump()
        axes = [joystick.get_axis(i) for i in range(joystick.get_numaxes())]
        buttons = [joystick.get_button(i) for i in range(joystick.get_numbuttons())]

        print("Axes:", ["{:.2f}".format(a) for a in axes], end=" | ")
        print("Buttons:", buttons, end="\r")

except KeyboardInterrupt:
    print("\nExiting.")
