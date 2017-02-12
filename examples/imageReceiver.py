# File:  imageReceiver.py
# Date:  2/11/2017
# Auth:  K. Loux

# This is the code that receives the image from the network.  Image processing will happen
# here. For now it returns dummy data.

import socket
import cv2
import numpy

printFrames = 25

# Socket parameters
hostIP = '127.0.0.1'
hostPort = 5801# must match server configuration
bufferSize = 1024

print("Running image receiver")

while True:
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:# Configure socket to use TCP/IP

        print('Waiting to connect to server')
        while True:
            try:
                s.connect((hostIP, hostPort))# Connect to the server
                break
            except ConnectionRefusedError:
                continue# Just keep trying - this error means the server isn't ready (or that you've configured the IP/port info incorrectly...
            except OSError:
                continue# Also keep trying - if we attempt to reconnect too soon after a failed connection, we can get this exception
        print('Connected, beginning image processing loop')
        frameCount = 0# For printing periodic diagnostic messages

        while True:
            # Initialize the variables required to decode the image data
            imageSize = 1
            imageData = ''
            while len(imageData) < imageSize:# Loop until we have the entire image
                try:
                    data = s.recv(bufferSize)# Get the data from the server (this will be just one message out of many required to build an image)
                    if not data: break
                except ConnectionResetError:
                    print('Lost connection to server during receive')
                    s.close()
                    break

                if len(imageData) == 0:# If this is our first time through the loop, read the message header so we know when to stop
                    # Decode the message - first read the size of the image (stored in the first 4 bytes)
                    imageSize = int.from_bytes(data[0:4], byteorder='big')
                    #print('started receiving image, size = ' + str(imageSize))
                    imageData = data[4:]# The rest of the message is image data
                else:
                    imageData += data# For subsequent messages, the entire payload is image data

            # Reconnect if we failed to get all of the image data
            if (len(imageData) < imageSize):
                break

            #print('final size = ' + str(len(image)))
            image = cv2.imdecode(numpy.fromstring(imageData, numpy.uint8), cv2.IMREAD_COLOR)
            #cv2.imwrite('test.png', image)# Uncomment to write an image file to confirm that transfer is good
            
            #
            #
            #
            #
            # TODO:  Image processing stuff goes here
            #
            #
            #
            #
            
            # Build dummy response string - eventually this can be useful information like location of a target, etc.
            # Server code needs to know how to interpret the message (because just printing a string to the screen won't be very helpful),
            # but the format of message can be anything we want, as long as the client and server both know the format.
            height, width, channels = image.shape
            responseString = 'Received image size = ' + str(height) + ' x ' + str(width) + ' x ' + str(channels) + ' (' + str(len(imageData)) + ' bytes)'
            
            # Print messages periodically, just so we know we're still alive
            frameCount += 1
            if frameCount % printFrames == 0:
                print('Processed ' + str(frameCount) + ' images since last connection')

            try:
                s.sendall(responseString.encode('utf-8'))
            except ConnectionResetError:
                print('Lost connection to server during send')
                s.close()
                break
