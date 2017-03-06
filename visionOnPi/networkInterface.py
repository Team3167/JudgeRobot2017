# File:  networkInterface.py
# Date:  3/6/2017
# Auth:  K. Loux
# Desc:  Network interface for one-way communication from Raspberry Pi to
#        RoboRio.

import socket
import struct

class networkInterface:
    def __init__(self, targetIP, targetPort):
        self.sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        #self.sock.setsockopt(socket.IPPROTO_IP, socket.IP_MULTICAST_TTL, 5)
        #self.sock.settimeout(timeout)
        self.targetIP = targetIP
        self.targetPort = targetPort
    
    def sendPosition(self, position):
        message = bytearray(struct.pack('>fff', position[0], position[1], position[2]))
        self.sock.sendto(message, (self.targetIP, self.targetPort))