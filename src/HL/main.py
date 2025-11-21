# States
# 0 - Idle
# 1 - Auto Driving
# 2 - Manual Driving with ps4 controller

from gpiozero import LED, Button, Buzzer
from luma.core.interface.serial import i2c
from luma.core.render import canvas
from luma.oled.device import ssd1306
from PIL import Image, ImageDraw, ImageFont
import struct
import smbus  # type: ignore #ignore the module could not be resolved error because it is a linux only module
import textwrap

from get_ip import get_ip, check_ssh_connections

from multiprocessing import Processus #pour pouvoir lancer les programme sur un cpu différent

bus = smbus.SMBus(1)  # 1 indicates /dev/i2c-1
#oled
serial = i2c(port=1, address=0x3C)
device = ssd1306(serial)

auto_driving_p = Process(target=Auto_Driving)
manual_driving_p = Process(target=Manual_Driving)

list_p = [auto_driving_p,manual_driving_p] #liste tout les processus possible

SLAVE_ADDRESS = 0x08  # I2C address of the slave arduino or stm32
bp_next = Button("GPIO5", bounce_time=0.1)
bp_entre = Button("GPIO6", bounce_time=0.1)
led1 = LED("GPIO17")
led2 = LED("GPIO27")
buzzer = Buzzer("GPIO26")
State = 0
Screen = 0
TEXT_HEIGHT = 11
TEXT_LEFT_OFFSET = 3 # Offset from the left of the screen to ensure no cuttoff


def make_voltage_im():
    try: #Will fail if arduino is rest ex: temporary power loss when plugu=ing in usb
        received = read_data(2)  # Adjust length as needed
    except OSError:
        received = [0.0, 0.0]
        print("I2C bus error")
    
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

def write_data(float_data):
    # Convert the float to bytes
    print(float_data)
    byte_data = struct.pack('f', float_data)
    # Convert the bytes to a list of integers
    int_data = list(byte_data)
    print(int_data)
    int_data.append(0)
    # Write the data to the I2C bus
    bus.write_i2c_block_data(SLAVE_ADDRESS, int_data[0], int_data[1:4])

def read_data(num_floats=3):

    # Each float is 4 bytes
    length = num_floats * 4
    # Read a block of data from the slave
    data = bus.read_i2c_block_data(SLAVE_ADDRESS, 0, length)
    # Convert the byte data to floats
    if len(data) >= length:
        float_values = struct.unpack('f' * num_floats, bytes(data[:length]))
        return list(float_values)
    else:
        raise ValueError("Not enough data received from I2C bus")


def Idle(): #Enable chossing between states
    global Screen
    global State
    if Screen==0 and check_ssh_connections():
        led1.on()
        Screen=1
    if not check_ssh_connections():
        led1.off()
    match Screen: #Display on OLED
        case 0: #IP and ssh status
            ip=get_ip()
            text = "Ready to SSH\nIP:"+ip
        case 1: #AutoDriving mode
            text = "Auto Driving"
        case 2: #Manual Driving mode
            text = "Manual Driving With PS4 Controller"
            #PS4 controller status
        case 3: #Kill all processus
            text = "Kill all the process"
    
    if (State==Screen):
        text+=": (en cours)"

    display_combined_im(text)


    if bp_next.is_pressed:
        bp_next.wait_for_release()
        Screen+=1
        if Screen>len(list_p):
            Screen=0
    if bp_entre.is_pressed:
        bp_entre.wait_for_release() 
        State=Screen
        p_choisie(State-1) # -1 because of the process cases begin at 1 and the list_p indexe at 0

def p_choisie(p_id):
    """ Kill tout les processus et lance celui choisie par p_id dans la liste list_p
        (Donner un p_id plus grand que len(list_p) tue tout les processus
    """
    for i in range(len(list_p)):
        if (i!= p_id):
            list_p[i].terminate()

    if (p_id<len(list_p)): #vérifie que le p_id soit valide dans
        if not list_[p_id].is_alive(): #evite de relancer le programme si déjà selectionée
            list_p[p_id].start()


def Auto_Driving():
    global State
    try:
        if Driving_has_not_started:
            from Car import Car
            GR86 = Car()
        GR86.main()
        if bp_entre.is_pressed or bp_next.is_pressed:
            raise KeyboardInterrupt
    except KeyboardInterrupt:
        GR86.stop()
        Driving_has_not_started = False
        State=0
        
        
def Manual_Driving():
    global State
    print("Manual Driving")
    State=0

def main():
    while True:
        Idle()
            
        
if __name__ == "__main__":
    main()