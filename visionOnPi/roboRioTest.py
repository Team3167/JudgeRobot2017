#! /usr/bin/env python 

import socket
import struct

def main():
    port = 5801# Must match port defined in piVisionMain
    
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.bind(('0.0.0.0', port))
    sock.setblocking(True)
    
    bufferSize = 128
    
    while True:
        result = sock.recvfrom(bufferSize)
        x, y, theta = struct.unpack('>fff', result[0])
        print('Received data from ' + result[1][0] +
        ' -> (' + str(x) + ', ' + str(y) + ', ' + str(theta) + ')\n')

if __name__ == '__main__':
    main()

        