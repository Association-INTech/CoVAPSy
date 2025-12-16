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
from Camera import Camera
from Autotech_constant import SOCKET_ADRESS, LIDAR_DATA_SIGMA, LIDAR_DATA_AMPLITUDE, LIDAR_DATA_OFFSET, SLAVE_ADDRESS

#différent programme
from scripts.commande_PS4 import PS4ControllerProgram
from SshProgramme import SshProgramme
from RemoteControl import RemoteControl
from Poweroff import Poweroff
from Camera import ProgramStreamCamera
from module_initialisation import Initialisation

serial = i2c(port=1, address=0x3C)
device = ssd1306(serial)
#on démarre les log
log.basicConfig(level=log.INFO)

bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1

 #le nombre de donnée récupéré par l'i2c


TEXT_HEIGHT = 11
TEXT_LEFT_OFFSET = 3 # Offset from the left of the screen to ensure no cuttoff


private = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
private.bind(("127.0.0.1", 5555))

# on utilise tcp pour les infos des différents informations
telemetry = context.socket(zmq.REP)
telemetry.bind("tcp://0.0.0.0:5557")


class Serveur():

    def __init__(self):
        #initialisation des différents module qui tourne tout le temps
        self.camera = Camera()

        self.bp_next = Button("GPIO5", bounce_time=0.1)
        self.bp_entre = Button("GPIO6", bounce_time=0.1)

        self.led1 = LED("GPIO17")
        self.led2 = LED("GPIO27")
        self.buzzer = Buzzer("GPIO26")
        self.remote_control = False # on initialise le remote control à False

        self.length_i2c_received = 3 #nombre de donnée à récupéré de l'arduino (voltage lipo, voltage nimh)
        
        # initialisation des donnnée de la voiture
        self.vitesse_d = 0 #vitesse demandé par le programme
        self.direction = 0 #direction des roue 

        #initialisation des variable reçue de l'arduino pour débugage
        self.voltage_lipo = 0
        self.voltage_nimh = 0
        self.vitesse_r = 0 #vitesse réel de la voiture

        self.camera_reverse = True

        # initialisation des commande de temps
        self.initial_time = time.time()
        self.last_cmd_time = time.time()

        #donnée des process
        self.process_output = ""
        self.last_programme_control = 0
        self.process = None
        self.temp = None

        self.initialisation_module = Initialisation(Camera,lidar,tof)

        
        @property
        def camera(self):
            return self.initialisation_module.camera

        @property
        def lidar(self):
            return self.initialisation_module.lidar
        
        @property
        def tof():
            return self.initialisation_module.tof

        self.programme = [SshProgramme(), self.initialisation_module, PS4ControllerProgram(), RemoteControl(), ProgramStreamCamera(self.camera), Poweroff()]

        # donnée de l'écran
        self.Screen = 0
        self.State = 0
        self.scroll_offset = 3

    #-----------------------------------------------------------------------------------------------------
    # affichage de l'écrans
    #-----------------------------------------------------------------------------------------------------
    def affichage_oled(self,selected): #test non utilisé
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
        """crée l'image de la derniére ligne qui affiche le voltage des deux batterie de la pi en temps réel"""
        
        received = [self.voltage_lipo , self.voltage_nimh]
        # filter out values below 6V and round to 2 decimal places
        received = [round(elem, 2) if elem > 6 else 0.0 for elem in received]
        text = f"LiP:{received[0]:.2f}V|NiH:{received[1]:.2f}V"
        im = Image.new("1", (128, TEXT_HEIGHT), "black")
        draw = ImageDraw.Draw(im)
        font = ImageFont.load_default()
        draw.text((3, 0), text, fill="white", font=font)
        return im

    def display_combined_im(self,text):
        """ fonction qui écris sur l'écran le texte qu'on lui fourni (et remet par dessus toujours le voltage des batteries)"""
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


    def Idle(self):
        """
        gére l'affichage de l'écrans en fonction des fonction en cour ou choisie.
        le changement d'écran est géré par les fonction des boutons juste en dessous
        """           
        if check_ssh_connections():
            self.led1.on()

        if not check_ssh_connections():
            self.led1.off()
        
        if (self.Screen < len(self.programme)):
            text = self.programme[self.Screen].display()
        self.display_combined_im(text)

    def bouton_next(self):
        """ passe à l'écrans suivant (juste visuelle)"""
        self.Screen+=1
        if self.Screen>=len(self.programme):
            self.Screen=0

    def bouton_entre(self,num=None):
        """séléctionne le programme afficher à l'acrans et le lance"""
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


    #---------------------------------------------------------------------------------------------------
    # fonction pour la communication
    #---------------------------------------------------------------------------------------------------
    def i2c_loop(self):
        """Envoie vitesse/direction régulièrement au microcontroleur. (toute les frames actuellement)"""
        print("lancement de l'i2c")
        while True:
            try :
                data = struct.pack('<ff', float(round(self.programme[self.last_programme_control].vitesse_d)), float(round(self.programme[self.last_programme_control].direction)))
                bus.write_i2c_block_data(SLAVE_ADDRESS, 0, list(data))
            except Exception as e:
                print("i2c mort" + str(e))
                time.sleep(1)

    def i2c_received(self):
        """récupére les informations de l'arduino"""
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

    def envoie_donnee(self, socket):
        """ on regarde si il s'agit de lappelle pour le control interne 
        (is_private) ou si on veux prendre le controle depuis le pc."""
        import base64
        from io import BytesIO
        while True:
            info = socket.recv_json()
            if info["get"] == "info":
                socket.send_json({
                "voltage_lipo": self.voltage_lipo,
                "voltage_nimh": self.voltage_nimh,
                "vitesse_reelle": self.vitesse_r,
                "vitesse_demande": self.vitesse_d,
                "direction" : self.direction,
                "timestamp": time.time() - self.initial_time
            })
            elif info["cmd"] == "menu":
                if info["menu"] in self.programme.key:
                    start_process(self,info["menu"]) #lancement du menu reçue
                    socket.send_json({"status":"ok"})
            elif info["get"] == "menu":
                socket.send_json(self.programme)
            else :
                socket.send_json({"Error" : "not understand"})

    def lidar_update_data(self):
        """donnée du lidar"""
        self._initialize_lidar()
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

        
    def start_process(self,num_programme):
        """lance le porgramme référencé avec son numéro:
        si il sagit d'un programme qui controle la voiture il kill lancient programme qui controlé,
        sinon le programme est lancé ou tué celon si il était déjà lancé ou tué avant"""
        if self.programme[num_programme].running:
            self.programme[num_programme].kill()
            if self.programme[num_programme].controls_car:
                self.last_programme_control = 0
            
        elif self.programme[num_programme].controls_car:
            self.programme[self.last_programme_control].kill()
            self.programme[num_programme].start()
            self.last_programme_control = num_programme
        
        else:
            self.programme[num_programme].start()
            

        
        


    #---------------------------------------------------------------------------------------------------
    # car function 
    #---------------------------------------------------------------------------------------------------



    def main(self):
        self.bp_next.when_pressed = self.bouton_next
        self.bp_entre.when_pressed = self.bouton_entre

        threading.Thread(target=self.i2c_loop, daemon=True).start()
        threading.Thread(target=self.i2c_received, daemon=True).start()
        threading.Thread(target=self.envoie_donnee, args=(telemetry,), daemon=True).start()
        threading.Thread(target=self.lidar_update_data, daemon=True).start()
        
        while True:
            self.Idle()

#---------------------------------------------------------------------------------------------------
# main
#---------------------------------------------------------------------------------------------------

if __name__ == "__main__":
    boot = Serveur()
    boot.main()
