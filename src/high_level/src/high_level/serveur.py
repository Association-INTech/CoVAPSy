import logging
import textwrap
import time

import smbus
from gpiozero import LED, Button, Buzzer
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont

from high_level.autotech_constant import SITE_DIR_BACKEND, TEXT_HEIGHT
from programs.car import AIProgram, CrashCar
from programs.initialisation import Initialisation
from programs.poweroff import Poweroff
from programs.ps4_controller_program import PS4ControllerProgram
from programs.remote_control import RemoteControl
from programs.ssh_programme import SshProgramme
from programs.utils.ssh import check_ssh_connections

from .backend import BackendAPI


class Serveur:
    def __init__(self):
        self.log = logging.getLogger(__name__)
        # initialization of different modules
        self.log.info("Server initialization")

        # initialization of GPIO buttons, LEDs, buzzer
        self.bp_next = Button("GPIO5", bounce_time=0.1)
        self.bp_entre = Button("GPIO6", bounce_time=0.1)

        self.led1 = LED("GPIO17")
        self.led2 = LED("GPIO27")
        self.buzzer = Buzzer("GPIO26")
        self.log.info("GPIO: boutons, LEDs, buzzer initialized")

        self.serial = i2c(port=1, address=0x3C)
        self.device = ssd1306(self.serial)
        self.bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1
        self.log.info("I2C: bus open on /dev/i2c-1")

        # initialization of time command
        self.initial_time = time.time()
        self.last_cmd_time = time.time()

        # data of the car
        self.process_output = ""
        self.last_program_control = 0
        self.process = None
        self.temp = None

        self.initialisation_module = Initialisation(self)
        self.crash_car = CrashCar(self.initialisation_module.lidar)

        self.programs = [
            SshProgramme(),
            self.initialisation_module,
            AIProgram(self),
            PS4ControllerProgram(),
            RemoteControl(),
            # ProgramStreamCamera(self),
            BackendAPI(self, host="0.0.0.0", port=8001, site_dir=SITE_DIR_BACKEND),
            Poweroff(),
        ]
        self.log.debug("Programs ready: %s", [type(p).__name__ for p in self.programs])

        # screen data
        self.screen = 0
        self.state = 0
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
        return self.programs[self.last_program_control].target_speed

    @property
    def direction(self):
        return self.programs[self.last_program_control].direction

    # -----------------------------------------------------------------------------------------------------
    # Screen display functions
    # -----------------------------------------------------------------------------------------------------
    def affichage_oled(self, selected: int):  # test not use
        im = Image.new("1", (128, 64), "black")
        draw = ImageDraw.Draw(im)
        font = ImageFont.load_default()

        for num, i in enumerate(
            range(
                max(selected - self.scroll_offset, 0),
                min(len(self.programs), selected + self.scroll_offset),
            )
        ):
            y = num * TEXT_HEIGHT

            if i == selected:
                draw.rectangle((0, y, 127, y + TEXT_HEIGHT), fill="white")
                draw.text((3, y), self.programs[i]["name"], fill="black", font=font)
            else:
                draw.text((3, y), self.programs[i]["name"], fill="white", font=font)

        with canvas(self.device) as display:
            display.bitmap((0, 0), im, fill="white")

    def make_voltage_im(self):
        """Create an image showing the battery voltages to be pasted on the main display"""
        if self.arduino_I2C is not None:
            received = [self.arduino_I2C.voltage_lipo, self.arduino_I2C.voltage_nimh]
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

    def display_combined_im(self, text: str):
        """function to display text with battery voltages on the oled screen"""
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

    def idle(self):
        """
        Manages the screen display based on the current or chosen function.
        Screen changes are managed by the button functions just below.
        """
        if check_ssh_connections():
            self.led1.on()

        if not check_ssh_connections():
            self.led1.off()

        if self.screen < len(self.programs):
            text = self.programs[self.screen].display()
            self.display_combined_im(text)

    def bouton_next(self):
        """go to next screen on oled display"""
        self.screen += 1
        if self.screen >= len(self.programs):
            self.screen = 0

    def bouton_entre(self, num=None):
        """take action on the current screen on display and start the program"""
        if num is not None:
            self.screen = num
        self.state = self.screen
        self.start_process(self.screen)

    # ---------------------------------------------------------------------------------------------------
    # Processus
    # ---------------------------------------------------------------------------------------------------

    def start_process(self, number_program):
        """Starts the program referenced by its number:
        if it is a program that controls the car, it kills the old program that was controlling,
        otherwise the program is started or stopped depending on whether it was already running or stopped before"""
        self.log.info(
            "User action: program %d (%s)",
            number_program,
            type(self.programs[number_program]).__name__,
        )
        if self.programs[number_program].running:
            self.programs[number_program].kill()
            if self.programs[number_program].controls_car:
                self.last_program_control = 0
                self.log.warning(
                    "Car control changed: %s -> %s",
                    type(self.programs[number_program]).__name__,
                    type(self.programs[self.last_program_control]).__name__,
                )

            self.log.info(
                "Program %s stopped", type(self.programs[number_program]).__name__
            )

        elif self.programs[number_program].controls_car:
            self.programs[self.last_program_control].kill()
            self.programs[number_program].start()
            self.log.warning(
                "Car control changed: %s -> %s",
                type(self.programs[self.last_program_control]).__name__,
                type(self.programs[number_program]).__name__,
            )
            self.last_program_control = number_program

        else:
            self.programs[number_program].start()

    # ---------------------------------------------------------------------------------------------------
    # car function
    # ---------------------------------------------------------------------------------------------------

    def main(self):
        self.bp_next.when_pressed = self.bouton_next
        self.bp_entre.when_pressed = self.bouton_entre

        self.log.info("Server main loop started")

        while True:
            self.idle()
