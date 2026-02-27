from actionneur_capteur import Lidar

IP = "192.168.0.10"
PORT = 10940


def main():
    # Create an instance of HokuyoReader
    lidar = Lidar(IP, PORT)

    # Stop any previous measurements
    lidar.stop()

    # Start a single read
    lidar.single_read(0, 1080)

    # Print the distance values
    print(lidar.r_distance)

    # Stop the lidar
    lidar.stop()


if __name__ == "__main__":
    main()
