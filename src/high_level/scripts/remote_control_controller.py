import socket
import struct
import time
from threading import Thread

import pygame

from high_level.autotech_constant import IP

IP = "192.168.1.21"

###################################################
# UDP communication
###################################################
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

direction = 0.0
target_speed = 0.0
program_running = True


def send_data():
    """Send the current speed and direction periodically."""
    global target_speed, direction, program_running

    while program_running:
        packet = struct.pack("ff", target_speed, direction)
        sock.sendto(packet, (IP, 5556))
        time.sleep(0.05)


###################################################
# Vehicle configuration
###################################################
max_target_speed = 2  # m/s
min_target_speed = -2  # m/s
angle_degree_max = 18  # degrees


def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def set_direction_degrees(angle_degrees):
    global direction
    direction = float(angle_degrees)


def set_target_speed(speed_mm_per_second):
    global target_speed
    target_speed = float(speed_mm_per_second)


###################################################
# Controller detection
###################################################
def detect_joystick():
    """
    Return the first available joystick.
    Return None if no controller is connected.
    """
    pygame.joystick.quit()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        return None

    joystick = pygame.joystick.Joystick(0)
    joystick.init()
    return joystick


###################################################
# Joystick control
###################################################
def update_from_joystick(joystick):
    """
    Read an Xbox / PS4-like USB controller.

    axis 0: left stick horizontal
    axis 2: left trigger
    axis 5: right trigger
    """
    axis_lx = joystick.get_axis(0)
    axis_l2 = joystick.get_axis(2)
    axis_r2 = joystick.get_axis(5)

    # Steering
    steering_angle = map_range(axis_lx, -1, 1, -angle_degree_max, angle_degree_max)
    set_direction_degrees(round(steering_angle))

    # Acceleration and reverse
    acceleration = (axis_r2 + 1) / 2
    reverse = (axis_l2 + 1) / 2

    if acceleration > 0.05:
        speed = acceleration * max_target_speed * 1000
        set_target_speed(round(speed))

    elif reverse > 0.05:
        speed = reverse * min_target_speed * 1000
        set_target_speed(round(speed))

    else:
        set_target_speed(0)


###################################################
# Keyboard control
###################################################
def update_from_keyboard():
    """
    Control the vehicle with arrow keys.

    Up: forward
    Down: reverse
    Left / Right: steering
    """
    keys = pygame.key.get_pressed()

    # Steering
    if keys[pygame.K_LEFT] and not keys[pygame.K_RIGHT]:
        set_direction_degrees(-angle_degree_max)

    elif keys[pygame.K_RIGHT] and not keys[pygame.K_LEFT]:
        set_direction_degrees(angle_degree_max)

    else:
        set_direction_degrees(0)

    # Speed
    if keys[pygame.K_UP] and not keys[pygame.K_DOWN]:
        set_target_speed(max_target_speed * 1000)

    elif keys[pygame.K_DOWN] and not keys[pygame.K_UP]:
        set_target_speed(min_target_speed * 1000)

    else:
        set_target_speed(0)


###################################################
# Main program
###################################################
if __name__ == "__main__":
    pygame.init()
    pygame.joystick.init()

    # A window is required for reliable keyboard input.
    screen = pygame.display.set_mode((520, 160))
    pygame.display.set_caption("Vehicle control")

    font = pygame.font.Font(None, 28)

    joystick = detect_joystick()

    if joystick is None:
        control_mode = "keyboard"
        print("No joystick detected: keyboard control enabled.")
        print("Use the arrow keys. Press Escape to quit.")

    else:
        control_mode = "joystick"
        print("Joystick detected:", joystick.get_name())

    Thread(target=send_data, daemon=True).start()

    last_detection_time = time.time()

    try:
        while program_running:
            for event in pygame.event.get():
                if event.type == pygame.QUIT:
                    program_running = False

                elif event.type == pygame.KEYDOWN:
                    if event.key == pygame.K_ESCAPE:
                        program_running = False

            ###################################################
            # Regularly check whether a controller was
            # connected or disconnected.
            ###################################################
            if time.time() - last_detection_time > 1:
                last_detection_time = time.time()

                available_joystick = detect_joystick()

                if available_joystick is None:
                    if control_mode != "keyboard":
                        print("Joystick disconnected: switching to keyboard.")
                    joystick = None
                    control_mode = "keyboard"

                else:
                    if control_mode != "joystick":
                        print("Joystick detected:", available_joystick.get_name())
                    joystick = available_joystick
                    control_mode = "joystick"

            ###################################################
            # Apply the appropriate control mode.
            ###################################################
            if control_mode == "joystick" and joystick is not None:
                try:
                    update_from_joystick(joystick)

                except pygame.error:
                    # Fallback if the controller disappears
                    # between two detection checks.
                    joystick = None
                    control_mode = "keyboard"
                    set_target_speed(0)
                    set_direction_degrees(0)

            else:
                update_from_keyboard()

            ###################################################
            # Display current state.
            ###################################################
            screen.fill((0, 0, 0))

            lines = [
                f"Mode: {control_mode}",
                f"Speed command: {target_speed:.0f} mm/s",
                f"Direction: {direction:.0f} degrees",
                "Arrow keys: drive | Escape: quit",
            ]

            for index, text in enumerate(lines):
                surface = font.render(text, True, (255, 255, 255))
                screen.blit(surface, (15, 15 + index * 32))

            pygame.display.flip()

            print(
                f"mode={control_mode}, "
                f"direction={direction:.0f}, "
                f"speed={target_speed:.0f}"
            )

            time.sleep(0.01)

    except KeyboardInterrupt:
        print("End of program.")

    finally:
        # Immediately send a stop command before closing.
        program_running = False
        set_target_speed(0)
        set_direction_degrees(0)

        packet = struct.pack("ff", target_speed, direction)
        sock.sendto(packet, (IP, 5556))

        sock.close()
        pygame.quit()
