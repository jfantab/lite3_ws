import socket 
import struct 
import os 

socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
ctrl_addr = ('192.168.1.120', 45678)

filename = b'mscs.wav'
socket.sendto(filename, ctrl_addr)
