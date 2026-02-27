import socket
import struct
import time
from threading import Thread

import pygame

###################################################
# Init ZMQ
###################################################
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)


def send_data():
    global target_speed, direction
    while True:
        packet = struct.pack("ff", target_speed, direction)
        sock.sendto(packet, ("192.168.1.10", 5556))
        time.sleep(0.05)


###################################################
# Vehicle control variables
###################################################
direction = 0
target_speed = 0

max_target_speed = 2
min_target_speed = -2
angle_degree_max = 18


def map_range(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min


def set_direction_degrees(angle_degrees):
    global direction
    direction = angle_degrees
    print(direction, target_speed)


def set_target_speed(vit):
    global target_speed
    target_speed = vit


if __name__ == "__main__":
    ###################################################
    # Init pygame + controller
    ###################################################
    pygame.init()
    pygame.joystick.init()

    if pygame.joystick.get_count() == 0:
        print("no joystick detected")
        exit(1)

    joy = pygame.joystick.Joystick(0)
    joy.init()
    print("joystick detected:", joy.get_name())

    ###################################################
    # Main loop
    ###################################################
    Thread(target=send_data, daemon=True).start()

    try:
        while True:
            pygame.event.pump()

            # Axes:
            # For Xbox/PS4 USB:
            # L2 = axis 2   (often 0..1)
            # R2 = axis 5   (often 0..1)
            # Left stick horizontal = axis 0 (-1..1)

            axis_lx = joy.get_axis(0)  # Left right
            axis_l2 = joy.get_axis(2)  # Reverse acceleration
            axis_r2 = joy.get_axis(5)  # Forward acceleration

            # Direction
            direction = map_range(axis_lx, -1, 1, -angle_degree_max, angle_degree_max)
            set_direction_degrees(round(direction))

            # Acceleration
            accel = (axis_r2 + 1) / 2
            brake = (axis_l2 + 1) / 2

            # Some controllers go from -1..1, others 0..1

            # Forward
            if accel > 0.05:
                vit = accel * max_target_speed * 1000
                set_target_speed(round(vit))

            # Reverse
            elif brake > 0.05:
                vit = brake * min_target_speed * 1000
                set_target_speed(round(vit))
            else:
                set_target_speed(0)
            time.sleep(0.01)

    except KeyboardInterrupt:
        print("End of program.")
        pygame.quit()
