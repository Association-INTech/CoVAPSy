# MIT License
#
# Copyright (c) 2020 cassc
#
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
#
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
#
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.


import _thread as thread
import logging
import os
import socket

import matplotlib.pyplot as plt
import numpy as np


class Lidar:
    measure_msg_heads = {"ME", "GE", "MD", "GD"}

    def deg_to_theta(self, deg):
        return deg / 360 * 2 * np.pi

    def make_socket(self, ip, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        sock.connect((ip, port))
        sock.settimeout(5)
        return sock

    # decode 3 byte integer data line
    def decode_distance(self, data):

        def partition(n: int, lst):
            for i in range(0, len(lst), n):
                yield lst[i : i + n]

        # remove checksum bytes for every 65 bytes of data
        parts = ["".join(part[:-1]) for part in list(partition(65, data))]
        # repack data
        data = "".join(parts)
        # transform 3-byte int
        ns = [ord(d) - 0x30 for d in data]
        ns = [f"{d:06b}" for d in ns]
        ns = list(partition(3, ns))
        ns = [int("".join(c3), 2) for c3 in ns]
        # set out-of-range value to zero
        ns = [0 if n == 65533 else n for n in ns]

        self.r_distance = np.array(ns)
        self.m_step = self.start_step
        self.measuring = False
        return ns

    def __init__(self, ip, port, start_step=0):
        self.log = logging.getLogger(__name__)
        self.ip = ip
        self.port = port
        # For decoding measuring data
        self.measuring = False
        self.skip = 0
        self.head = None
        # Starting step id
        self.start_step = start_step
        # Current step id, for decoding polar distance data
        self.m_step = start_step
        # Array of distance
        self.r_distance = np.zeros(1081 - start_step, dtype=int)
        # Buffer to receive packets
        self.buf = ""
        self.expected_packet_size = (
            65 * 50 + 44
        )  # TODO hardcoded for full range measurement

        ids = np.arange(1081 - start_step)
        self.x_theta = self.deg_to_theta((ids + start_step) * 270.0 / 1080 + 45 - 90)

        self.sock = self.make_socket(ip, port)
        self.__start_reader__()

    def send(self, cmd: str):
        self.sock.sendall(cmd.encode())

    def start_plotter(self, autorange=False):

        def to_cartesian(x_theta, x_r):
            x = np.cos(x_theta) * x_r
            y = np.sin(x_theta) * x_r
            return x, y

        plt.show()
        plt.figure()
        axc = plt.subplot(121)
        axp = plt.subplot(122, projection="polar")
        # axp.set_thetamax(deg2theta(45))
        # axp.set_thetamax(deg2theta(270 + 45))
        axp.grid(True)
        self.log.info("Plotter started, press any key to exit")

        self.log.debug(f"{self.x_theta}, {self.r_distance}")
        while True:
            x, y = to_cartesian(self.x_theta, self.r_distance)

            axp.clear()
            axc.clear()

            axp.plot(self.x_theta, self.r_distance)

            axc.plot(x, y)

            if not autorange:
                axp.set_rmax(8000)
                axc.set_xlim(-5000, 5000)
                axc.set_ylim(-5000, 5000)

            plt.pause(1e-17)

            if plt.waitforbuttonpress(timeout=0.02):
                os._exit(0)

    # Change hokuyo IP address, requires reboot
    def change_ip(self, ip: str, gateway: str, netmask="255.255.255.0"):
        def format_zeros(addr):
            return "".join([n.rjust(3, "0") for n in addr.split(".")])

        ip = format_zeros(ip)
        gateway = format_zeros(gateway)
        netmask = format_zeros(netmask)
        cmd = f"$IP{ip}{netmask}{gateway}\r\n"
        self.log.debug(f"ChangeIP cmd:  {cmd}")
        self.send(cmd)

    # Start continous read mode
    def start_continuous(self, start: int, end: int, with_intensity=False):
        head = "ME" if with_intensity else "MD"
        cmd = f"{head}{start:04d}{end:04d}00000\r\n"
        self.log.debug(cmd)
        self.head = cmd.strip()
        self.send(cmd)

    # Start single read
    def single_read(self, start: int, end: int, with_intensity=False):
        head = "GE" if with_intensity else "GD"
        cmd = f"{head}{start:04d}{end:04d}01000\r\n"
        self.send(cmd)

    def stop(self):
        cmd = "QT\r\n"
        self.send(cmd)

    def reboot(self):
        cmd = "RB\r\n"
        self.send(cmd)
        self.send(cmd)

    def handle_msg_line(self, line):
        if line == self.head:
            self.measuring = True
            self.skip = 0
            self.m_step = self.start_step
            return True

        if self.measuring:
            if self.skip < 2:
                self.skip += 1
                return True
            else:
                self.buf += line.strip()
                # self.log.debug(f'buf size {len(self.buf)}')
                if len(self.buf) >= self.expected_packet_size:
                    self.decode_distance(self.buf)
                    self.buf = ""
                return True

        return False

    def __start_reader__(self):
        def handle_measuring(msg):
            lines = msg.split()
            for line in lines:
                if not self.handle_msg_line(line):
                    self.log.debug(f"ignore {line}")

        def loop():
            try:
                while True:
                    try:
                        m, _ = self.sock.recvfrom(1024)
                        msg = m.decode()
                        handle_measuring(msg)
                    except socket.timeout:
                        self.log.error("Read timeout, sensor disconnected?")
                        os._exit(1)
            finally:
                self.sock.close()

        thread.start_new_thread(loop, ())
