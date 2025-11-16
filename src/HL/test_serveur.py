"""
import zerorpc
import time

c = zerorpc.Client()
c.connect(f"tcp://0.0.0.0:4242")


if __name__ == "__main__":
    while(True):
        vitesse= float(input("vitesse en millimetre par seconde:"))
        rotation= float(input("rotation en degré:"))
        c.write_vitesse_direction(vitesse,rotation)
        time.sleep(0.1)  # Wait for the slave to process the data
        received = c.read_data(3)  # Adjust length as needed
        print("Received from slave:", received[0], received[1], received[2] )

        # Request data from the slave"""

import zmq
import time
# on envoie les données au serveur
context = zmq.Context()
socket = context.socket(zmq.REQ)
socket.connect("tcp://192.168.1.10:5555")

def envoie_donnee(vitesse,rotation):
    socket.send_json({"cmd": "set_speed", "value": vitesse})
    resp = socket.recv_json()
    socket.send_json({"cmd": "set_direction", "value": rotation})
    resp = socket.recv_json()
def recoit_donnee():
    socket.send_json({"cmd": "info"})
    resp = socket.recv_json()
    print(resp)

if __name__ == "__main__":
    while(True):
        
        vitesse= float(input("vitesse en millimetre par seconde:"))
        rotation= float(input("rotation en degré:"))
        envoie_donnee(vitesse,rotation)
        recoit_donnee()
        time.sleep(0.1)  # Wait for the slave to process the data+ù

        # Request data from the slave