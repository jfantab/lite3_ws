import socket 
import struct 
import os 

def tts_file(file):
    socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM, 0)
    ctrl_addr = ('192.168.1.120', 45678)

    socket.bind(('192.168.1.65', 54321))

    filename = bytes(file, 'utf-8')
    socket.sendto(filename, ctrl_addr)

    msg, addr = socket.recvfrom(1024)
    result = msg.decode('utf-8')
    return result