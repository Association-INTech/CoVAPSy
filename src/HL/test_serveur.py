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
        received = c.read_data(8)  # Adjust length as needed
        print("Received from slave:", received)

        # Request data from the slave