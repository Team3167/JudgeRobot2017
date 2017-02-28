# File:  mockRoboRio.py
# Date:  2/11/2017
# Auth:  K. Loux

# This is the mock RoboRio - it captures images from the camera and sends them to the client.
# The RoboRio runs at approximately 50 Hz.  The camera, however, may only provide data at 30 Hz
# (or less, depending on configuration).  Although images can be captured from the camera using
# FIRST libraries, for this test script we use OpenCV, which means that the computer running
# this script also requires OpenCV.  It could be run from the Raspberry Pi, if that's easiest.
# Refer to 2017 FRC Rule R65.

# We will transmit this data using TCP.  The other option is UDP.  The difference is that TCP
# performs more error checking and can detect dropped packets.  If a packet is lost, it will
# be re-sent, so we can be sure the entire image will be transmitted every time (because it
# will be streamed instead of sent as a large chunck).  UDP can be faster, because it doesn't
# perform these extra checks, but for now, I think we should assume we want TCP.  I think that
# it would be difficult to use UDP to transmit images like this.

# In TCP, there is a server and a client.  The sockets on each end get configured differently.
# We'll assume that the RoboRio will be the server.

# Other things to consider - get image directly from camera?  h.264 stream available from RoboRio?

# See bandwidth information here:  https://wpilib.screenstepslive.com/s/fms/m/whitepaper/l/608744-fms-whitepaper

import socket
import time
import cv2

# Simulation parameters
loopFrequency = 30.0# [Hz]
loopPeriod = 1.0 / loopFrequency# [sec]
printFrames = 25

# Socket parameters
hostPort = 5801# must be in the range 5800-5810
bufferSize = 100# this can be tuned - typically a larger value is used, but we may want a smaller
#                 value in order to get a faster response (i.e. improve latency at the expense
#                 of speed, I think).

# Questions:
# 1.  Do I need to pay attention to the port on which spawnned sockets operate?
# 2.  Tradeoff between sending a compressed image (processor time) and sending uncompressed
#     image (network bandwidth)
# 3.  Similarly, need to decide between resizing/converting to greyscale, etc. on RoboRio vs
#     Raspberry Pi.  I think the only good way to answer these questions is with testing.
# 4.  Will we need to use a separate thread on the RoboRio so we keep the rest of the robot
#     operating quickly? (I think yes)
# 5.  Can we improve performance by having the RoboRio grab camera images in parallel with
#     waiting for a response from the Raspberry Pi?  Currently, we don't attempt to get the
#     next image until after we've got a response from the Rapsberry Pi - maybe not the ideal
#     design.

# Camera parameters
cameraPort = 0# May need to change?

print("Running RoboRio Simulation")

camera = cv2.VideoCapture(cameraPort)
if not camera.isOpened():
    print('Failed to open camera - check cameraPort\n')
    exit(1)

# Helper method for getting the current image from the camera
def getImage():
    _, image = camera.read()
    return image

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:# Configure socket to use TCP/IP
    s.bind(('', hostPort))# Assign the socket to the desired port

    while True:# If the connection drops, we go back to this loop and wait for another connection
        print('Waiting for client to connect...')
        s.listen(1)# Wait for a client to connect - this blocks, meaning that the program will hang here until
        #            a client does connect.  The argument is the maximum number of connections we'll accept.

        conn, addr = s.accept()# Accept the connection - returns a new socket (conn) and the client address
        #                        All future communication with the client occurs through the conn socket and
        #                        not the s socket (s socket exists only as a means to establish new connections).
        with conn:
            print('New connection with ', addr)
            
            frameCount = 0
            statStart = time.time()
            sentBytes = 0
            recvBytes = 0
            latency = []
            
            while True:
                start = time.time()
                image = cv2.imencode('.png', getImage())[1].tostring()# Encode the image in an easy-to-transmit format
                #print('sending image of size = ' + str(len(image)))

                # In order to ensure the client knows how to receive the image, the first four bytes of the messate
                # tell the client the total image size (different from message size, since we added information to the
                # message).  So the client will continue to receive messages until it has received the entire image.
                message = (len(image)).to_bytes(4, byteorder='big')
                message += image

                try:
                    # This stuff is in a try block so that the application doesn't crash in the event of a disconnection
                    sendTime = time.time()
                    conn.sendall(message)
                    sentBytes += len(message)
                    response = conn.recv(bufferSize)# Wait for a response from the client - this will also block
                    latency.append(time.time() - sendTime)
                    recvBytes += len(response)
                    
                    # Print the response message as a check to ensure it's working properly
                    #print('Got response: ' + response.decode('utf-8'))
                except ConnectionResetError:
                    print('Client connection reset')
                    break
                except BrokenPipeError:
                    print('Client connection reset')
                    break
                if not response: break
                
                frameCount += 1
                if frameCount % printFrames == 0:
                    statEnd = time.time()
                    deltaTime = statEnd - statStart
                    print('\nEstimated bandwidth usage = ' + str(round((sentBytes + recvBytes) / deltaTime * 1.0e-6, 2)) + ' Mbps')
                    print('Average latency           = ' + str(round(sum(latency) / len(latency) * 0.001, 1)) + ' ms')
                    print('Average framerate         = ' + str(round(frameCount / deltaTime, 2)) + ' Hz')
                    
                    frameCount = 0# Reset this so we always have a "recent framerate" instead of average since we started
                    statStart = statEnd
                    sentBytes = 0
                    recvBytes = 0
                    latency = []
                    
                # We want to do this a specific rate - so don't just run as fast as we can - sleep to maintain timing if we need to
                elapsed = time.time() - start
                if (elapsed < loopPeriod):
                    time.sleep(loopPeriod - elapsed)

del(camera)# Free the camera to prevent problems with repeated executions of this script
