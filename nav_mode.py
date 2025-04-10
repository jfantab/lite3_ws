import threading
import time
import socket
import struct
import os
from dotenv import load_dotenv

load_dotenv()

HEARTBEAT = int(os.getenv("HEARTBEAT"), 0)
ZERO = int(os.getenv("ZERO"), 0)
STAND_UP_LIE_DOWN = int(os.getenv("STAND_UP_LIE_DOWN"), 0)
NAV_MODE = int(os.getenv("NAV_MODE"), 0)

class Auto():

    def __init__(self, local_port=20001, ctrl_ip="192.168.1.120", ctrl_port=43893):
        self.local_port = local_port
        self.server = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
        self.ctrl_addr = (ctrl_ip, ctrl_port)

        # self.socket_lock = threading.Lock()
        # self.stop_event = threading.Event()
        self.back = threading.Thread(name="background", target=self.background)
        
    def send_command(self, command, param1=0, param2=0):
        if self.server._closed:  # Check if socket is usable
            print("Socket connection closed")
            return False

        packet = struct.pack('<3i', command, param1, param2)
        try:
            self.server.sendto(packet, self.ctrl_addr)
        except (OSError, struct.error) as e:
            print("Failure sending command: ", {str(e)})

    def background(self):
        # while not self.stop_event.is_set():
        while True:
            self.send_command(HEARTBEAT)
            time.sleep(0.5)
        print("Background thread stopping...")

    def initialize(self):
        # if self.back and self.back.is_alive():
        #     print("Background thread running...")
        #     return
        print("Initializing...")

        # self.stop_event.clear()
        self.back.start()

        self.send_command(ZERO)
        time.sleep(2)

        self.send_command(STAND_UP_LIE_DOWN)
        time.sleep(2)
        self.send_command(NAV_MODE)
        time.sleep(2)

    def shutdown(self):
        print("Shutting down!")
        # self.send_command(STAND_UP_LIE_DOWN)
        
        # self.stop_event.set()
        # if self.back:
        #     self.back.join(timeout=2)

if __name__ == "__main__":    
    a = Auto()
    a.initialize()
    # try: 
    #     a.initialize()
    #     while True:
    #         time.sleep(1)
    # except KeyboardInterrupt:
    #     a.shutdown()

