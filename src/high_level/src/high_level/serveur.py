import time
import smbus
import logging

from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont
from gpiozero import LED, Button, Buzzer
import textwrap

from programs.utils.ssh import check_ssh_connections
from actionneur_capteur.lidar import Lidar
from actionneur_capteur.camera import Camera
from actionneur_capteur.tof import ToF
from actionneur_capteur.masterI2C import I2c_arduino

#différent programme
from programs.ps4_controller_program import PS4ControllerProgram
from programs.ssh_programme import SshProgramme
from programs.remote_control import RemoteControl
from programs.poweroff import Poweroff
from actionneur_capteur.camera import ProgramStreamCamera
from programs.initialisation import Initialisation
from programs.car import Ai_Programme
from backend import BackendAPI

from high_level.autotech_constant import TEXT_HEIGHT

class Serveur():

    def __init__(self):
        self.log = logging.getLogger(__name__)
        #initialisation des différents module qui tourne tout le temps
        self.log.info("Initialisation du serveur")

        # initialisation des boutons et de l'i2c
        self.bp_next = Button("GPIO5", bounce_time=0.1)
        self.bp_entre = Button("GPIO6", bounce_time=0.1)

        self.led1 = LED("GPIO17")
        self.led2 = LED("GPIO27")
        self.buzzer = Buzzer("GPIO26")
        self.log.info("GPIO: boutons, LEDs, buzzer initialisés")
        
        self.serial = i2c(port=1, address=0x3C)
        self.device = ssd1306(self.serial)
        self.bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1
        self.log.info("I2C: bus ouvert sur /dev/i2c-1")

        # initialisation des commande de temps
        self.initial_time = time.time()
        self.last_cmd_time = time.time()

        #donnée des process
        self.process_output = ""
        self.last_programme_control = 0
        self.process = None
        self.temp = None

        self.initialisation_module = Initialisation(self,Camera,Lidar,ToF, I2c_arduino)
        
        self.programme = [SshProgramme(),
                         self.initialisation_module, 
                         Ai_Programme(self), 
                         PS4ControllerProgram(), 
                         RemoteControl(), 
                         ProgramStreamCamera(self), 
                         BackendAPI(self, host="0.0.0.0", port=8001, site_dir="/home/intech/CoVAPSy/src/HL/site_controle"),
                         Poweroff()]
        self.log.debug("Programmes chargés: %s", [type(p).__name__ for p in self.programme])

        # donnée de l'écran
        self.Screen = 0
        self.State = 0
        self.scroll_offset = 3


    @property
    def camera(self):
        return self.initialisation_module.camera

    @property
    def lidar(self):
        return self.initialisation_module.lidar
    
    @property
    def tof(self):
        return self.initialisation_module.tof

    @property
    def arduino_I2C(self):
        return self.initialisation_module.arduino_I2C
    
    @property
    def target_speed(self):
        return self.programme[self.last_programme_control].target_speed
    
    @property
    def direction(self):
        return self.programme[self.last_programme_control].direction
        
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

        with canvas(self.device) as display:
            display.bitmap((0, 0), im, fill="white")

    def make_voltage_im(self):
        """crée l'image de la derniére ligne qui affiche le voltage des deux batterie de la pi en temps réel"""
        if self.arduino_I2C is not None:
            received = [self.arduino_I2C.voltage_lipo , self.arduino_I2C.voltage_nimh]
        else:
            received = [0.0, 0.0]

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
        
        with canvas(self.device) as draw:
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
    # Processus
    #---------------------------------------------------------------------------------------------------

        
    def start_process(self,num_programme):
        """lance le porgramme référencé avec son numéro:
        si il sagit d'un programme qui controle la voiture il kill lancient programme qui controlé,
        sinon le programme est lancé ou tué celon si il était déjà lancé ou tué avant"""
        self.log.info("Action utilisateur: programme %d (%s)",
            num_programme,
            type(self.programme[num_programme]).__name__)
        if self.programme[num_programme].running:
            self.programme[num_programme].kill()
            if self.programme[num_programme].controls_car:
                self.last_programme_control = 0
                self.log.warning(
                "Changement de contrôle voiture: %s -> %s",
                    type(self.programme[num_programme]).__name__,
                    type(self.programme[self.last_programme_control]).__name__
                )
                
            
            self.log.info("Arrêt du programme %s",
            type(self.programme[num_programme]).__name__)
            
        elif self.programme[num_programme].controls_car:
            self.programme[self.last_programme_control].kill()
            self.programme[num_programme].start()
            self.log.warning(
                "Changement de contrôle voiture: %s -> %s",
                    type(self.programme[self.last_programme_control]).__name__,
                    type(self.programme[num_programme]).__name__
                )
            self.last_programme_control = num_programme

        
        else:
            self.programme[num_programme].start()
            

    #---------------------------------------------------------------------------------------------------
    # car function 
    #---------------------------------------------------------------------------------------------------

    def main(self):
        self.bp_next.when_pressed = self.bouton_next
        self.bp_entre.when_pressed = self.bouton_entre

        self.log.info("Serveur démarré, entrée dans la boucle principale")

        while True:
            self.Idle()

