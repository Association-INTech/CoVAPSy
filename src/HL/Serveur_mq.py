import zmq
context = zmq.Context()

import time
import threading
import smbus
import logging as log
import struct

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont
from gpiozero import LED, Button, Buzzer
import textwrap

from get_ip import get_ip, check_ssh_connections
import subprocess
serial = i2c(port=1, address=0x3C)
device = ssd1306(serial)
#on démarre les log
log.basicConfig(level=log.INFO)

bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

# I2C address of the slave
SLAVE_ADDRESS = 0x08

length_i2c_received = 3 #le nombre de donnée récupéré par l'i2c

bp_next = Button("GPIO5", bounce_time=0.1)
bp_entre = Button("GPIO6", bounce_time=0.1)
led1 = LED("GPIO17")
led2 = LED("GPIO27")
buzzer = Buzzer("GPIO26")
TEXT_HEIGHT = 11
TEXT_LEFT_OFFSET = 3 # Offset from the left of the screen to ensure no cuttoff

# on recoit les inoformations
received = context.socket(zmq.REP)
received.bind("tcp://0.0.0.0:5555")

vitesse_d = 0
vitesse_r = 0
direction = 0

voltage_lipo = 0
voltage_nimh = 0

initial_time = time.time()
last_cmd_time = time.time()

ip = get_ip()


process_output = ""
programme = {
    0: {
        "name" : "Ssh to :\n" + ip,
        "type" : "",
        "path" : "",
        "info" : ""
    },
    1: {
        "name" : "Auto Driving",
        "type" : "python",
        "path" : "",
        "info" : ""
    },
    2: {
        "name" : "PS4 Controller",
        "type" : "python",
        "path" : "./scripts/commande_PS4.py",
        "info" : ""
    },
    3: {
        "name" : "Connect Controller",
        "type" : "bash",
        "path" : "./scripts/bluetooth_auto/bluethootconnect.sh",
        "info" : ""
    },
    4: {
        "name" : "Kill all",
        "type" : "",
        "path" : "",
        "info" : ""
    }
}

Screen = 0
State = 0
#-----------------------------------------------------------------------------------------------------
# fonction utile
#-----------------------------------------------------------------------------------------------------
def make_voltage_im():
    global voltage_lipo, voltage_nimh
    received = [voltage_lipo , voltage_nimh]  # Adjust length as needed
    # filter out values below 6V and round to 2 decimal places
    received = [round(elem, 2) if elem > 6 else 0.0 for elem in received]
    text = f"LiP:{received[0]:.2f}V|NiH:{received[1]:.2f}V"
    im = Image.new("1", (128, TEXT_HEIGHT), "black")
    draw = ImageDraw.Draw(im)
    font = ImageFont.load_default()
    draw.text((3, 0), text, fill="white", font=font)
    return im

def display_combined_im(text):
    im = Image.new("1", (128, 64), "black")
    draw = ImageDraw.Draw(im)
    font = ImageFont.load_default()
    
    # Wrap the text to fit within the width of the display
    wrapped_text = textwrap.fill(text, width=20)  # Adjust width as needed
    draw.text((3, 0), wrapped_text, fill="white", font=font)
    
    voltage_im = make_voltage_im()
    im.paste(voltage_im, (0, 64 - TEXT_HEIGHT))
    
    with canvas(device) as draw:
        draw.bitmap((0, 0), im, fill="white")


def Idle(): #Enable chossing between states
    global Screen
    global State
    if Screen==0 and check_ssh_connections():
        led1.on()
        Screen=1
    if not check_ssh_connections():
        led1.off()
    
    if (Screen <= len(programme)):
        text = programme[Screen]["name"] + "\n" + process_output

    display_combined_im(text)

    if bp_next.is_pressed:
        bp_next.wait_for_release()
        Screen+=1
        if Screen>=len(programme):
            Screen=0
    if bp_entre.is_pressed:
        bp_entre.wait_for_release() 
        State=Screen
        start_process(Screen)


#---------------------------------------------------------------------------------------------------
# fonction pour la communication
#---------------------------------------------------------------------------------------------------
def i2c_loop():
    """Envoie vitesse/direction régulièrement au microcontroleur."""
    global vitesse_d, direction, last_cmd_time

    while True:
        try :
            if (time.time()- last_cmd_time < 0.2):
                data = struct.pack('<ff', float(vitesse_d), float(direction))
                bus.write_i2c_block_data(SLAVE_ADDRESS, 0, list(data))
                time.sleep(0.05)
            else: # on renvoie zero si il on a pas recue de message depuis moins de 200 milisecondes
                vitesse_d = 0
                direction = 0
                data = struct.pack('<ff', float(vitesse_d), float(direction))
                bus.write_i2c_block_data(SLAVE_ADDRESS, 0, list(data))
                time.sleep(0.05)
        except :
            print("i2c mort")
            time.sleep(1)

def i2c_received():
    
    global voltage_lipo, voltage_nimh, vitesse_r, length_i2c_received
    length = length_i2c_received * 4 
    while True:
        data = bus.read_i2c_block_data(SLAVE_ADDRESS, 0, length)
        # Convert the byte data to a float
        if len(data) >= length:
            float_values = struct.unpack('f' * length_i2c_received, bytes(data[:length]))
            list_valeur = list(float_values)

            # on enregistre les valeur
            voltage_lipo = list_valeur[0]
            voltage_nimh = list_valeur[1]
            vitesse_r = list_valeur[2]
        time.sleep(0.1)



def msg_received():
    global vitesse_d, direction, last_cmd_time
    while True :
        req = received.recv_json()

        if req["cmd"] == "set_speed":
            vitesse_d = req["value"]
            received.send_json({"status": "ok"})
            last_cmd_time = time.time()

        elif req["cmd"] == "set_direction":
            direction = req["value"]
            received.send_json({"status": "ok"})
            last_cmd_time = time.time()

        elif req["cmd"] == "info":
            received.send_json({
            "voltage_lipo": voltage_lipo,
            "voltage_nimh": voltage_nimh,
            "vitesse_reelle": vitesse_r,
            "timestamp": time.time() - initial_time
        })
        else:
            received.send_json({"error": "unknown"})


#---------------------------------------------------------------------------------------------------
# Processus
#---------------------------------------------------------------------------------------------------
def stream_process_output(proc):
    global process_output
    for line in proc.stdout:
        process_output = line.decode().strip()
    lines = proc.stdout.split("\n")
    size = 3
    chunks = [l[i * size : (i+1) * size] for l in lines for i in range(len(l) // size + 1)]
    print(chunks)
    
def start_process(num_programme):
    global process, programme, process_output
    try :
        process.kill()
    except :
        pass

    programme_actuel = programme[num_programme]
    if programme_actuel["type"] == "bash":
        process = subprocess.Popen(
            programme_actuel["path"],
            shell=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT
        )
    elif programme_actuel["type"] == "python":
        process = subprocess.Popen(["uv","run",programme_actuel["path"]])


    process_output = ""
    threading.Thread(target=stream_process_output, args=(process,), daemon=True).start()
#---------------------------------------------------------------------------------------------------
# main
#---------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    threading.Thread(target=i2c_loop, daemon=True).start()
    threading.Thread(target=i2c_received, daemon=True).start()
    threading.Thread(target=msg_received, daemon=True).start()
    
    while True:
        Idle()
