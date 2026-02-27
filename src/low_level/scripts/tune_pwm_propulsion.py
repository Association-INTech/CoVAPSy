import time

from rpi_hardware_pwm import HardwarePWM

# parameters for the speed_m_s function, to be calibrated
direction_prop = (
    1.0  # 1 for controllers on the right or a small ratio corresponds to forward drive
)
pwm_stop_prop = 1.0
point_mort_prop = 1.0
delta_pwm_max_prop = 1.0  # pwm at which maximum speed is reached

vitesse_max_m_s_hard = 8  # maximum speed the car can achieve
vitesse_max_m_s_soft = 2  # maximum speed we want to achieve

pwm_prop = HardwarePWM(
    pwm_channel=0, hz=50, chip=2
)  # use chip 2 on pi 5 in accordance with the documentation
pwm_prop.start(pwm_stop_prop)


def set_speed_m_s(vitesse_m_s):
    if vitesse_m_s > vitesse_max_m_s_soft:
        vitesse_m_s = vitesse_max_m_s_soft
    elif vitesse_m_s < -vitesse_max_m_s_hard:
        vitesse_m_s = -vitesse_max_m_s_hard
    if vitesse_m_s == 0:
        pwm_prop.change_duty_cycle(pwm_stop_prop)
    elif vitesse_m_s > 0:
        vitesse = vitesse_m_s * (delta_pwm_max_prop) / vitesse_max_m_s_hard
        pwm_prop.change_duty_cycle(
            pwm_stop_prop + direction_prop * (point_mort_prop + vitesse)
        )
    elif vitesse_m_s < 0:
        vitesse = vitesse_m_s * (delta_pwm_max_prop) / vitesse_max_m_s_hard
        pwm_prop.change_duty_cycle(
            pwm_stop_prop - direction_prop * (point_mort_prop - vitesse)
        )


def reverse():
    set_speed_m_s(-vitesse_max_m_s_hard)
    time.sleep(0.2)
    set_speed_m_s(0)
    time.sleep(0.2)
    set_speed_m_s(-1)


print("Adjust stops, Q to quit")
print("Enter numeric value to test a speed in mm/s")
print("R to reverse")
print("I to reverse left and right")
print("p to decrease delta_pwm_max_prop and P to increase it")
print("z to decrease the zero point 1.5 ms and Z to increase it")
print("m to decrease the neutral point and M to increase it")


while True:
    a = input("speed in mm/s, R, I, p, P, z, Z, m, M: ")
    try:
        vitesse_mm_s = int(a)
        set_speed_m_s(vitesse_mm_s / 1000.0)
    except Exception:
        if a == "I" or a == "i":
            direction_prop = -direction_prop
            print("new direction: " + str(direction_prop))
        elif a == "R":
            reverse()
            print("reverse")
        elif a == "p":
            delta_pwm_max_prop -= 0.1
            print("new delta_pwm_max_prop: " + str(delta_pwm_max_prop))
            pwm_prop.change_duty_cycle(
                pwm_stop_prop + direction_prop * (point_mort_prop + delta_pwm_max_prop)
            )
        elif a == "P":
            delta_pwm_max_prop += 0.1
            print("new delta_pwm_max_prop: " + str(delta_pwm_max_prop))
            pwm_prop.change_duty_cycle(
                pwm_stop_prop + direction_prop * (point_mort_prop + delta_pwm_max_prop)
            )
        elif a == "z":
            pwm_stop_prop -= 0.01
            print("new pwm_stop_prop: " + str(pwm_stop_prop))
            pwm_prop.change_duty_cycle(pwm_stop_prop)
        elif a == "Z":
            pwm_stop_prop += 0.01
            print("new pwm_stop_prop: " + str(pwm_stop_prop))
            pwm_prop.change_duty_cycle(pwm_stop_prop)
        elif a == "m":
            point_mort_prop -= 0.01
            print("new point_mort_prop: " + str(point_mort_prop))
            pwm_prop.change_duty_cycle(
                pwm_stop_prop + direction_prop * (point_mort_prop)
            )
        elif a == "M":
            point_mort_prop += 0.01
            print("new point_mort_prop: " + str(point_mort_prop))
            pwm_prop.change_duty_cycle(
                pwm_stop_prop + direction_prop * (point_mort_prop)
            )
        else:
            break

pwm_prop.change_duty_cycle(pwm_stop_prop)
print("new values")
print("direction: " + str(direction_prop))
print("delta_pwm_max_prop: " + str(delta_pwm_max_prop))
print("zero point 1.5 ms: " + str(pwm_stop_prop))
print("neutral point: " + str(point_mort_prop))
