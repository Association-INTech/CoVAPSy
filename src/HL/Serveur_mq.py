import zmq
context = zmq.Context()

import time
import threading
import smbus
import logging as log
import struct
import os, signal


from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont
from gpiozero import LED, Button, Buzzer
import textwrap
import socket

from get_ip import get_ip, check_ssh_connections
import subprocess
from Lidar import Lidar
from Camera import Camera
from Autotech_constant import SOCKET_ADRESS, LIDAR_DATA_SIGMA, LIDAR_DATA_AMPLITUDE, LIDAR_DATA_OFFSET

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
"""
private = context.socket(zmq.SUB)
private.bind("tcp://127.0.0.1:5555")
private.setsockopt_string(zmq.SUBSCRIBE, "")

public = context.socket(zmq.SUB)
public.bind("tcp://0.0.0.0:5556")
public.setsockopt_string(zmq.SUBSCRIBE, "")
"""
# on envoie en udp les commandes de la ps4
public = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
public.bind(("0.0.0.0", 5556))

private = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
private.bind(("127.0.0.1", 5555))

# on utilise tcp pour les infos des différents informations
telemetry = context.socket(zmq.REP)
telemetry.bind("tcp://127.0.0.1:5557")


class Serveur():

    def __init__(self):
        self.remote_control = False # on initialise le remote control à False

        self.vitesse_d = 0
        self.vitesse_r = 0
        self.direction = 0

        self.voltage_lipo = 0
        self.voltage_nimh = 0

        self.initial_time = time.time()
        self.last_cmd_time = time.time()

        self.ip = get_ip()

        #donnée des process
        self.process_output = ""
        self.last_programme = 0
        self.process = None
        self.programme = {
            0: {
                "name" : "Ssh to :\n" + ip,
                "type" : "",
                "path" : "",
                "info" : "no"
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
                "info" : "no"
            },
            4: {
                "name" : "Remote control",
                "type" : "function",
                "path" : lambda: switch_remote_control(),
                "info" : ""
            },
            5: {
                "name" : "poweroff",
                "type" : "bash",
                "path" : "sudo poweroff",
                "info" : ""
            }
        }

        #donnée du lidar
        self.lidar = None
        self.rDistance = []
        self.xTheta = 0

        # donnée de l'écran
        self.Screen = 0
        self.State = 0
        self.scroll_offset = 3

        #-----------------------------------------------------------------------------------------------------
        # affichage de l'écrans
        #-----------------------------------------------------------------------------------------------------
    def affichage_oled(self,selected): #test
        im = Image.new("1", (128, 64), "black")
        draw = ImageDraw.Draw(im)
        font = ImageFont.load_default()

        for num, i in enumerate(range(max(selected - self.scroll_offset, 0), min(len(programme), selected + scroll_offset))):
            y = num * TEXT_HEIGHT

            if i == selected:
                draw.rectangle((0, y, 127, y + TEXT_HEIGHT), fill="white")
                draw.text((3, y), programme[i]["name"], fill="black", font=font)
            else:
                draw.text((3, y), programme[i]["name"], fill="white", font=font)

        with canvas(device) as display:
            display.bitmap((0, 0), im, fill="white")

    def make_voltage_im(slef):
        received = [self.voltage_lipo , self.voltage_nimh]  # Adjust length as needed
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


    def Idle(self): #Enable chossing between states            
        if self.Screen==0 and check_ssh_connections():
            led1.on()
            slef.Screen=1
        if not check_ssh_connections():
            led1.off()
        
        if (self.Screen <= len(programme)):
            if programme[self.Screen]["info"] != "no" : 
                text = programme[self.Screen]["name"] + "\n" + programme[self.Screen]["info"] + "\n" + process_output
            else : 
                text = programme[self.Screen]["name"] + "\n" + process_output

        display_combined_im(text)

    def bouton_next(self):
        self.Screen+=1
        if self.Screen>=len(programme):
            self.Screen=0

    def bouton_entre(self,num=None):
        if num!=None:
            self.Screen = num
        self.State=self.Screen
        start_process(self.Screen)

    #---------------------------------------------------------------------------------------------------
    # initialisation
    #---------------------------------------------------------------------------------------------------

    def _initialize_lidar(self):
        """Initialize the Lidar sensor."""
        try:
            self.lidar = Lidar(SOCKET_ADRESS["IP"], SOCKET_ADRESS["PORT"])
            self.lidar.stop()
            self.lidar.startContinuous(0, 1080)
            print("Lidar initialized successfully")
        except Exception as e:
            print(f"Error initializing Lidar: {e}")

    def _initialize_camera(self):
        """Initialize the camera."""
        try:
            reverse_count = 0
            self.camera = Camera()
            self.camera.start()
            time.sleep(0.2)  # Allow time for the camera to start
            log.info("Camera initialized successfully")
        except Exception as e:
            log.error(f"Error initializing Camera: {e}")
            raise

    #---------------------------------------------------------------------------------------------------
    # fonction pour la communication
    #---------------------------------------------------------------------------------------------------
    def i2c_loop(self):
        """Envoie vitesse/direction régulièrement au microcontroleur."""
        global vitesse_d, direction, last_cmd_time

        while True:
            try :
                
                if (self.time.time()- self.last_cmd_time < 0.5):
                    data = struct.pack('<ff', float(round(vitesse_d)), float(round(direction)))
                    bus.write_i2c_block_data(SLAVE_ADDRESS, 0, list(data))
                    #time.sleep(0.00005)
                else: # on renvoie zero si il on a pas recue de message depuis moins de 200 milisecondes
                    self.vitesse_d = 0
                    self.direction = 0
                    data = struct.pack('<ff', float(self.vitesse_d), float(self.direction))
                    bus.write_i2c_block_data(SLAVE_ADDRESS, 0, list(data))
                    time.sleep(0.01)
            except :
                print("i2c mort")
                time.sleep(1)

    def i2c_received(self):
        length = self.length_i2c_received * 4 
        while True:
            data = bus.read_i2c_block_data(SLAVE_ADDRESS, 0, length)
            # Convert the byte data to a float
            if len(data) >= length:
                float_values = struct.unpack('f' * self.length_i2c_received, bytes(data[:length]))
                list_valeur = list(float_values)

                # on enregistre les valeur
                self.voltage_lipo = list_valeur[0]
                self.voltage_nimh = list_valeur[1]
                self.vitesse_r = list_valeur[2]
            time.sleep(0.1)



    def car_controle(self,socket, is_private):
        """ on regarde si il s'agit de lappelle pour le control interne 
        (is_private) ou si on veux prendre le controle depuis le pc."""

        while is_private or self.remote_control:
            data, ip = socket.recvfrom(1024)
            self.vitesse_d, self.direction = struct.unpack("ff", data)
            self.last_cmd_time = time.time()
            print(ip)

    def envoie_donnee(self, socket):
        """ on regarde si il s'agit de lappelle pour le control interne 
        (is_private) ou si on veux prendre le controle depuis le pc."""
        
        while True:
            info = socket.recv_json()
            if info["cmd"] == "info":
                socket.send_json({
                "voltage_lipo": self.voltage_lipo,
                "voltage_nimh": self.voltage_nimh,
                "vitesse_reelle": self.vitesse_r,
                "direction" : self.direction,
                "timestamp": time.time() - self.initial_time
            })
            else :
                socket.send_json({"Error" : "not understand"})

    def lidar_update_data(self):
        _initialize_lidar()
        while True:
            try :
                self.rDistance = self.lidar.rDistance
                self.xTheta = self.lidar.xTheta
                time.sleep(0.1)
            except :
                print("pas lidar")
                time.sleep(1)
    #---------------------------------------------------------------------------------------------------
    # Processus
    #---------------------------------------------------------------------------------------------------
    def stream_process_output(self, proc):
        for line in proc.stdout:
            self.process_output = line.decode().strip()
        """
        lines = proc.stdout.split("\n")
        size = 3
        chunks = [l[i * size : (i+1) * size] for l in lines for i in range(len(l) // size + 1)]
        print(chunks)"""
        
    def start_process(self,num_programme):
        global process, programme, process_output, last_programme

        if self.programme[self.last_programme]["info"] != "no":
            self.programme[self.last_programme]["info"] = ""
        
        if self.process is not None:
            try:
                os.killpg(os.getpgid(process.pid), signal.SIGTERM)
            except Exception as e:
                print(e)

        if (num_programme == self.last_programme):
            self.last_programme = 0 # pour pouvoir lancer le programme en rapuyant sur le bouton
            return # si on est sur le même programme on kill et c'est tout
        
        if self.programme[num_programme]["info"] != "no":
            self.programme[num_programme]["info"] = "(running)"
        self.process_output = ""
        self.last_programme = num_programme
        self.programme_actuel = self.programme[num_programme]
        if self.programme_actuel["type"] == "bash":
            process = subprocess.Popen(
                self.programme_actuel["path"],
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )

            threading.Thread(target=stream_process_output, args=(process,), daemon=True).start()
        elif self.programme_actuel["type"] == "python":
            process = subprocess.Popen(["uv","run",programme_actuel["path"]],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )

            threading.Thread(target=stream_process_output, args=(process,), daemon=True).start()
        elif self.programme_actuel["type"] == "function":
            self.programme_actuel["path"]()

        
        


    #---------------------------------------------------------------------------------------------------
    # car function 
    #---------------------------------------------------------------------------------------------------

    def switch_remote_control(self):
        if self.remote_control:
            self.remote_control = False
            programme[self.last_programme]["info"] = ""
        else:
            self.remote_control = True
        threading.Thread(target=car_controle, args=(public,False,), daemon=True).start()

    def main(self):
        self.bp_next.when_pressed = bouton_next
        self.bp_entre.when_pressed = bouton_entre

        threading.Thread(target=self.i2c_loop, daemon=True).start()
        threading.Thread(target=self.i2c_received, daemon=True).start()
        threading.Thread(target=self.car_controle, args=(private,True,), daemon=True).start()
        threading.Thread(target=self.envoie_donnee, args=(telemetry,), daemon=True).start()
        threading.Thread(target=self.lidar_update_data, daemon=True).start()
        
        while True:
            Idle()

#---------------------------------------------------------------------------------------------------
# main
#---------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    boot = Serveur()
    boot.main()
