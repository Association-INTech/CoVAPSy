from rpi_hardware_pwm import HardwarePWM

# startup parameters, with stops very close to the center
direction = -1  # 1 for angle_pwm_min to the left, -1 for angle_pwm_min to the right
angle_pwm_min = 6.91  # min
angle_pwm_max = 10.7  # max
angle_pwm_centre = 8.805

angle_degre_max = +18  # towards the left
angle_degre = 0

pwm_dir = HardwarePWM(
    pwm_channel=1, hz=50, chip=2
)  # use chip 2 on pi 5 in accordance with the documentation
pwm_dir.start(angle_pwm_centre)


def set_direction_degrees(angle_degrees):
    global angle_pwm_min
    global angle_pwm_max
    global angle_pwm_centre
    angle_pwm = angle_pwm_centre + direction * (
        angle_pwm_max - angle_pwm_min
    ) * angle_degrees / (2 * angle_degre_max)
    if angle_pwm > angle_pwm_max:
        angle_pwm = angle_pwm_max
    if angle_pwm < angle_pwm_min:
        angle_pwm = angle_pwm_min
    pwm_dir.change_duty_cycle(angle_pwm)


print("Adjust stops, Q to quit")
print("Enter numeric value to test a direction angle")
print("I to reverse left and right")
print("g to decrease the left stop and G to increase it")
print("d to decrease the right stop and D to increase it")

while True:
    a = input("angle, I, g, G, d, D ?")
    try:
        angle_degre = int(a)
        set_direction_degrees(angle_degre)
    except Exception:
        if a == "I":
            direction = -direction
            print("new direction: " + str(direction))
        elif a == "g":
            if direction == 1:
                angle_pwm_max -= 0.1
                print("new left stop: " + str(angle_pwm_max))
            else:
                angle_pwm_min += 0.1
                print("new left stop: " + str(angle_pwm_min))
            angle_pwm_centre = (angle_pwm_max + angle_pwm_min) / 2
            set_direction_degrees(18)
        elif a == "G":
            if direction == 1:
                angle_pwm_max += 0.1
                print("new left stop: " + str(angle_pwm_max))
            else:
                angle_pwm_min -= 0.1
                print("new left stop: " + str(angle_pwm_min))
            angle_pwm_centre = (angle_pwm_max + angle_pwm_min) / 2
            set_direction_degrees(18)
        elif a == "d":
            if direction == -1:
                angle_pwm_max -= 0.1
                print("new right stop: " + str(angle_pwm_max))
            else:
                angle_pwm_min += 0.1
                print("new right stop: " + str(angle_pwm_min))
            angle_pwm_centre = (angle_pwm_max + angle_pwm_min) / 2
            set_direction_degrees(-18)
        elif a == "D":
            if direction == -1:
                angle_pwm_max += 0.1
                print("new right stop: " + str(angle_pwm_max))
            else:
                angle_pwm_min -= 0.1
                print("new right stop: " + str(angle_pwm_min))
            angle_pwm_centre = (angle_pwm_max + angle_pwm_min) / 2
            set_direction_degrees(-18)
        else:
            break

print("new values")
print("direction: " + str(direction))
print("angle_pwm_min: " + str(angle_pwm_min))
print("angle_pwm_max: " + str(angle_pwm_max))
print("angle_pwm_centre: " + str(angle_pwm_centre))
