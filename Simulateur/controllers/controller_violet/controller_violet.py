# Copyright 1996-2022 Cyberbotics Ltd.
#
# Controle de la voiture TT-02 simulateur CoVAPSy pour Webots 2023b
# Inspiré de vehicle_driver_altino controller
# Kévin Hoarau, Anthony Juton, Bastien Lhopitallier, Martin Taynaud
# juillet 2023


import numpy as np
from vehicle import Driver
from controller import Lidar

driver = Driver()

basicTimeStep = int(driver.getBasicTimeStep())
sensorTimeStep = 4 * basicTimeStep

#Lidar
lidar = Lidar("Hokuyo")
lidar.enable(sensorTimeStep)
lidar.enablePointCloud()

touch_sensor = driver.getDevice("touch_sensor")

# vitesse en km/h
speed = 0
maxSpeed = 28 #km/h

# angle de la direction
angle = 0
maxangle = 0.28 #rad (étrange, la voiture est défini pour une limite à 0.31 rad...

# mise a zéro de la vitesse et de la direction
driver.setSteeringAngle(angle)
driver.setCruisingSpeed(speed)

while driver.step() != -1:
    speed = driver.getTargetCruisingSpeed()
    donnees_lidar = lidar.getRangeImage()
    sensor_data = touch_sensor.getValue()

    # goes backwards
    if sensor_data == 1:
        backwards_duration = 1000 # ms
        for _ in range(backwards_duration / basicTimeStep)
            speed = -1
            driver.setCruisingSpeed(speed)
            driver.setSteeringAngle(0)
            driver.step()


    speed = 3 #km/h
    #l'angle de la direction est la différence entre les mesures des rayons
    #du lidar à (-99+18*2)=-63° et (-99+81*2)=63°
    angle = donnees_lidar[60]-donnees_lidar[300]

    # clamp speed and angle to max values
    if speed > maxSpeed:
        speed = maxSpeed
    elif speed < -1 * maxSpeed:
        speed = -1 * maxSpeed
    if angle > maxangle:
        angle = maxangle
    elif angle < -maxangle:
        angle = -maxangle

    driver.setCruisingSpeed(speed)
    driver.setSteeringAngle(angle)
