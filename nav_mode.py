import threading
import time
import socket
import struct
import os
from dotenv import load_dotenv

load_dotenv()

# TODO: fix thread locking issues

HEARTBEAT = int(os.getenv("HEARTBEAT"), 0)
ZERO = int(os.getenv("ZERO"), 0)
STAND_UP_LIE_DOWN = int(os.getenv("STAND_UP_LIE_DOWN"), 0)
NAV_MODE = int(os.getenv("NAV_MODE"), 0)

class Auto():

    def __init__(self, local_port=20001, ctrl_ip="192.168.1.120", ctrl_port=43893):
        self.local_port = local_port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        self.ctrl_addr = (ctrl_ip, ctrl_port)
        self.initialize()

    def send_command(self, command, param1=0, param2=0):
        try:
            packet = struct.pack('<3i', command, param1, param2)
            self.server.sendto(packet, self.ctrl_addr)
        except Exception as e:
            print("Failure sending command: ", {e})

    def background(self):
        while True:
            self.send_command(HEARTBEAT)
            time.sleep(0.5)

    def initialize(self):
        self.back = threading.Thread(name="background", target=self.background)
        self.back.start()

        self.send_command(ZERO)
        self.send_command(STAND_UP_LIE_DOWN)
        self.send_command(NAV_MODE)
        
a = Auto()

