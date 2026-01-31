from high_level import Lidar

import time
from rpi_hardware_pwm import HardwarePWM

IP = '192.168.0.10'
PORT = 10940

# parameters for speed_m_s function
drive_direction = 1  # -1 for inverted ESCs or if a small ratio corresponds to forward motion
pwm_stop_drive = 7.53
dead_zone_drive = 0.5
delta_pwm_max_drive = 1.1  # PWM value at which maximum speed is reached

max_speed_m_s_hard = 8   # physical maximum speed of the car
max_speed_m_s_soft = 2   # desired maximum speed


# parameters for set_steering_degree function
steering_direction = 1  # 1 for min PWM angle to the left, -1 for min PWM angle to the right
steering_pwm_min = 6.91
steering_pwm_max = 10.7
steering_pwm_center = 8.805

max_steering_angle_deg = +18  # to the left
steering_angle_deg = 0

pwm_drive = HardwarePWM(pwm_channel=0, hz=50, chip=2)  # chip 2 on Pi 5 per documentation
pwm_drive.start(pwm_stop_drive)

def set_speed_m_s(speed_m_s):
    if speed_m_s > max_speed_m_s_soft:
        speed_m_s = max_speed_m_s_soft
    elif speed_m_s < -max_speed_m_s_hard:
        speed_m_s = -max_speed_m_s_hard

    if speed_m_s == 0:
        pwm_drive.change_duty_cycle(pwm_stop_drive)
    elif speed_m_s > 0:
        speed = speed_m_s * delta_pwm_max_drive / max_speed_m_s_hard
        pwm_drive.change_duty_cycle(
            pwm_stop_drive + drive_direction * (dead_zone_drive + speed)
        )
    elif speed_m_s < 0:
        speed = speed_m_s * delta_pwm_max_drive / max_speed_m_s_hard
        pwm_drive.change_duty_cycle(
            pwm_stop_drive - drive_direction * (dead_zone_drive - speed)
        )

def reverse():
    set_speed_m_s(-max_speed_m_s_hard)
    time.sleep(0.2)
    set_speed_m_s(0)
    time.sleep(0.2)
    set_speed_m_s(-1)

pwm_steering = HardwarePWM(pwm_channel=1, hz=50, chip=2)  # chip 2 on Pi 5 per documentation
pwm_steering.start(steering_pwm_center)

def set_steering_degree(angle_deg):
    global steering_pwm_min
    global steering_pwm_max
    global steering_pwm_center

    steering_pwm = (
        steering_pwm_center
        + steering_direction
        * (steering_pwm_max - steering_pwm_min)
        * angle_deg
        / (2 * max_steering_angle_deg)
    )

    if steering_pwm > steering_pwm_max:
        steering_pwm = steering_pwm_max
    if steering_pwm < steering_pwm_min:
        steering_pwm = steering_pwm_min

    pwm_steering.change_duty_cycle(steering_pwm)

# lidar connection and startup
lidar = Lidar(IP, PORT)
lidar.stop()
lidar.startContinuous(0, 1080)

lidar_table_mm = [0] * 360  # create a table of 360 zeros

time.sleep(1)  # lidar startup time

try:
    while True:
        for angle in range(len(lidar_table_mm)):
            # translation of lidar angle to table angle
            if angle > 135 and angle < 225:
                lidar_table_mm[angle] = float('nan')
            else:
                lidar_table_mm[angle] = lidar.rDistance[540 + (-angle * 4)]

        # steering angle is the difference between lidar rays
        # measured at -60 and +60 degrees
        steering_angle_deg = 0.02 * (lidar_table_mm[60] - lidar_table_mm[-60])
        print(lidar_table_mm[60], lidar_table_mm[-60], steering_angle_deg)

        set_steering_degree(steering_angle_deg)
        speed_m_s = 0.05
        set_speed_m_s(speed_m_s)

        time.sleep(0.1)
        ##############################################
except KeyboardInterrupt:  # catch CTRL+C
    speed_m_s = 0
    set_speed_m_s(speed_m_s)
    print("end of acquisitions")

# stop and disconnect lidar and motors
lidar.stop()
pwm_steering.stop()
pwm_drive.start(pwm_stop_drive)
