import zmq
context = zmq.Context()
import numpy as np
import cv2
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
from Camera import Camera, start_camera_stream
from Autotech_constant import SOCKET_ADRESS, LIDAR_DATA_SIGMA, LIDAR_DATA_AMPLITUDE, LIDAR_DATA_OFFSET


serial = i2c(port=1, address=0x3C)
device = ssd1306(serial)
#on démarre les log
log.basicConfig(level=log.INFO)

bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

# I2C address of the slave
SLAVE_ADDRESS = 0x08

 #le nombre de donnée récupéré par l'i2c


TEXT_HEIGHT = 11
TEXT_LEFT_OFFSET = 3 # Offset from the left of the screen to ensure no cuttoff

#sudo apt install libcap-dev
#sudo apt install python3-libcamera

#sudo apt-get install libcap-dev pour lancer picamera2
#rm -rf .venv
#uv venv --system-site-packages
#source .venv/bin/activate
#uv pip uninstall numpy

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
telemetry.bind("tcp://0.0.0.0:5557")


class Serveur():

    def __init__(self):
        self.bp_next = Button("GPIO5", bounce_time=0.1)
        self.bp_entre = Button("GPIO6", bounce_time=0.1)
        self.length_i2c_received = 3
        self.led1 = LED("GPIO17")
        self.led2 = LED("GPIO27")
        self.buzzer = Buzzer("GPIO26")
        self.remote_control = False # on initialise le remote control à False

        self.vitesse_d = 0
        self.vitesse_r = 0
        self.direction = 0

        self.voltage_lipo = 0
        self.voltage_nimh = 0
        self.camera_reverse = True

        self.initial_time = time.time()
        self.last_cmd_time = time.time()

        self.ip = get_ip()

        #donnée des process
        self.process_output = ""
        self.last_programme = 0
        self.process = None
        self.programme = {
            0: {
                "name" : "Ssh to :\n" + self.ip,
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
                "path" : lambda: self.switch_remote_control(),
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
        self.camera = Camera()

        #-----------------------------------------------------------------------------------------------------
        # affichage de l'écrans
        #-----------------------------------------------------------------------------------------------------
    def affichage_oled(self,selected): #test
        im = Image.new("1", (128, 64), "black")
        draw = ImageDraw.Draw(im)
        font = ImageFont.load_default()

        for num, i in enumerate(range(max(selected - self.scroll_offset, 0), min(len(self.programme), selected + self.scroll_offset))):
            y = num * TEXT_HEIGHT

            if i == selected:
                draw.rectangle((0, y, 127, y + TEXT_HEIGHT), fill="white")
                draw.text((3, y), self.programme[i]["name"], fill="black", font=font)
            else:
                draw.text((3, y), self.programme[i]["name"], fill="white", font=font)

        with canvas(device) as display:
            display.bitmap((0, 0), im, fill="white")

    def make_voltage_im(self):
        received = [self.voltage_lipo , self.voltage_nimh]  # Adjust length as needed
        # filter out values below 6V and round to 2 decimal places
        received = [round(elem, 2) if elem > 6 else 0.0 for elem in received]
        text = f"LiP:{received[0]:.2f}V|NiH:{received[1]:.2f}V"
        im = Image.new("1", (128, TEXT_HEIGHT), "black")
        draw = ImageDraw.Draw(im)
        font = ImageFont.load_default()
        draw.text((3, 0), text, fill="white", font=font)
        return im

    def display_combined_im(self,text):
        im = Image.new("1", (128, 64), "black")
        draw = ImageDraw.Draw(im)
        font = ImageFont.load_default()
        
        # Wrap the text to fit within the width of the display
        wrapped_text = textwrap.fill(text, width=20)  # Adjust width as needed
        draw.text((3, 0), wrapped_text, fill="white", font=font)
        
        voltage_im = self.make_voltage_im()
        im.paste(voltage_im, (0, 64 - TEXT_HEIGHT))
        
        with canvas(device) as draw:
            draw.bitmap((0, 0), im, fill="white")


    def Idle(self): #Enable chossing between states            
        if self.Screen==0 and check_ssh_connections():
            self.led1.on()
            self.Screen=1
        if not check_ssh_connections():
            self.led1.off()
        
        if (self.Screen < len(self.programme)):
            if self.programme[self.Screen]["info"] != "no" : 
                text = self.programme[self.Screen]["name"] + "\n" + self.programme[self.Screen]["info"] + "\n" + self.process_output
            else : 
                text = self.programme[self.Screen]["name"] + "\n" + self.process_output

        self.display_combined_im(text)

    def bouton_next(self):
        self.Screen+=1
        if self.Screen>=len(self.programme):
            self.Screen=0

    def bouton_entre(self,num=None):
        if num!=None:
            self.Screen = num
        self.State=self.Screen
        self.start_process(self.Screen)

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
        for i in range(10):
            try:
                self.camera = Camera()
                print("Camera OK")
                return
            except Exception as e:
                print("Camera retry", i, e)
                time.sleep(0.5)
        raise RuntimeError("Camera KO après 10 essais")

    #---------------------------------------------------------------------------------------------------
    # fonction pour la communication
    #---------------------------------------------------------------------------------------------------
    def i2c_loop(self):
        """Envoie vitesse/direction régulièrement au microcontroleur."""
        print("lancement de l'i2c")
        while True:
            try :
                
                if (time.time()- self.last_cmd_time < 0.5):
                    data = struct.pack('<ff', float(round(self.vitesse_d)), float(round(self.direction)))
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

    def envoie_donnee(self, socket):
        """ on regarde si il s'agit de lappelle pour le control interne 
        (is_private) ou si on veux prendre le controle depuis le pc."""
        import base64
        from io import BytesIO
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
            elif info["cmd"] == "cam":
                if self.camera_image is None:
                    socket.send_json({"cam": None})
                    continue

                buffer = BytesIO()
                self.camera_image.save(buffer, format="JPEG")
                jpg_bytes = buffer.getvalue()
                jpg_b64 = base64.b64encode(jpg_bytes).decode()

                socket.send_json({
                    "cam": jpg_b64
                })
                continue

            else :
                socket.send_json({"Error" : "not understand"})

    def lidar_update_data(self):
        self._initialize_lidar()
        while True:
            try :
                self.rDistance = self.lidar.rDistance
                self.xTheta = self.lidar.xTheta
                time.sleep(0.1)
            except :
                print("pas lidar")
                time.sleep(1)


    def _start_video_stream(self):
        """Start continuous JPEG compressed video streaming via ZMQ."""
        try:
            start_camera_stream(port=8000)
        except:
            print("Camera Down...")
    
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

        if self.programme[self.last_programme]["info"] != "no":
            self.programme[self.last_programme]["info"] = ""
        
        if self.process is not None:
            try:
                os.killpg(os.getpgid(self.process.pid), signal.SIGTERM)
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
            self.process = subprocess.Popen(
                self.programme_actuel["path"],
                shell=True,
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )

            threading.Thread(target=self.stream_process_output, args=(self.process,), daemon=True).start()
        elif self.programme_actuel["type"] == "python":
            self.process = subprocess.Popen(["uv","run",self.programme_actuel["path"]],
                stdout=subprocess.PIPE,
                stderr=subprocess.STDOUT,
                preexec_fn=os.setsid
            )

            threading.Thread(target=self.stream_process_output, args=(self.process,), daemon=True).start()
        elif self.programme_actuel["type"] == "function":
            self.programme_actuel["path"]()

        
        


    #---------------------------------------------------------------------------------------------------
    # car function 
    #---------------------------------------------------------------------------------------------------

    def switch_remote_control(self):
        if self.remote_control:
            self.remote_control = False
            self.programme[self.last_programme]["info"] = ""
        else:
            self.remote_control = True
        threading.Thread(target=self.car_controle, args=(public,False,), daemon=True).start()

    def main(self):
        self.bp_next.when_pressed = self.bouton_next
        self.bp_entre.when_pressed = self.bouton_entre

        threading.Thread(target=self.i2c_loop, daemon=True).start()
        threading.Thread(target=self.i2c_received, daemon=True).start()
        threading.Thread(target=self.car_controle, args=(private,True,), daemon=True).start()
        threading.Thread(target=self.envoie_donnee, args=(telemetry,), daemon=True).start()
        threading.Thread(target=self.lidar_update_data, daemon=True).start()
        threading.Thread(target=self._start_video_stream, daemon=True).start()

        while True:
            self.Idle()

#---------------------------------------------------------------------------------------------------
# main
#---------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    boot = Serveur()
    boot.main()
