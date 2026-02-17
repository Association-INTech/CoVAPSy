# Copyright 1996-2022 Cyberbotics Ltd.
#
# Control of the TT-02 car simulator CoVAPSy for Webots 2023b
# Inspired by vehicle_driver_altino controller
# Kévin Hoarau, Anthony Juton, Bastien Lhopitallier, Martin Raynaud
# August 2023

from controller import Lidar
from vehicle import Driver

driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

# Lidar
lidar = Lidar("Hokuyo")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud()

# clavier
keyboard = driver.getKeyboard()
keyboard.enable(sensorTimeStep)

# speed in km/h
speed = 0
maxSpeed = 28  # km/h

# steering angle
angle = 0
maxangle_degre = 16


# reset speed and direction to zero
driver.setSteeringAngle(angle)
driver.setCruisingSpeed(speed)

tableau_lidar_mm = [0.0] * 360


def set_speed_m_s(speed_m_s):
    speed = speed_m_s * 3.6
    if speed > maxSpeed:
        speed = maxSpeed
    if speed < 0:
        speed = 0
    driver.setCruisingSpeed(speed)


def set_direction_degrees(angle_degrees):
    if angle_degrees > maxangle_degre:
        angle_degrees = maxangle_degre
    elif angle_degrees < -maxangle_degre:
        angle_degrees = -maxangle_degre
    angle = -angle_degrees * 3.14 / 180
    driver.setSteeringAngle(angle)


def reverse():  # on the real car, there is a stop then a reverse for 1 second.
    driver.setCruisingSpeed(-1)


# auto mode disabled
modeAuto = False
print("Click on the 3D view to start")
print("a for auto mode (no manual mode on TT02_jaune), n for stop")

while driver.step() != -1:
    while True:
        # lidar data acquisition
        # keyboard key retrieval
        currentKey = keyboard.getKey()

        if currentKey == -1:
            break

        elif currentKey == ord("n") or currentKey == ord("N"):
            if modeAuto:
                modeAuto = False
                print("--------Auto Mode TT-02 Yellow Disabled-------")
        elif currentKey == ord("a") or currentKey == ord("A"):
            if not modeAuto:
                modeAuto = True
                print("------------Auto Mode TT-02 Yellow Enabled-----------------")

    # lidar data acquisition
    raw_lidar_data = lidar.getRangeImage()
    for i in range(360):
        if (raw_lidar_data[-i] > 0) and (raw_lidar_data[-i] < 20):
            tableau_lidar_mm[i] = 1000 * raw_lidar_data[-i]
        else:
            tableau_lidar_mm[i] = 0

    if not modeAuto:
        set_direction_degrees(0)
        set_speed_m_s(0)

    if modeAuto:
        ########################################################
        # Student program with
        #    - the tableau_lidar_mm array
        #    - the set_direction_degrees(...) function
        #    - the set_speed_m_s(...) function
        #    - the reverse() function
        #######################################################

        # one sector per 20° slice, so 10 sectors numbered 0 to 9
        angle_degre = 0.02 * (tableau_lidar_mm[60] - tableau_lidar_mm[-60])
        set_direction_degrees(angle_degre)
        speed_m_s = 0.5
        set_speed_m_s(speed_m_s)

    #########################################################
