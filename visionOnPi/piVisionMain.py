#! /usr/bin/env python
# File:  piVisionmain.py
# Date:  3/6/2017
# Auth:  K. Loux
# Desc:  Entry point for Raspberry Pi + OpenCV image processing.

from networkInterface import networkInterface
import time# TODO:  Remove

# The coordinate system we'll use here is as follows (diagram is view from
# above):
#
#      Target (gear pin)
#       ____
#        |
#        o ------------> positive Y-direction
#        |     _
#        |    / \
#        |    \  \ Robot
#        \/    \_/
#    Positive
#    X-direction
#
# For the given robot location and orientation (front of robot is point at
# target), this method should return a positive x value, a positive y value, and
# a positive orientation value.  Orientation is zero when the robot's forward/
# backwards directions is parallel to the X-axis.  Positive orientation means
# that the robot has rotated counter-clockwise away from zero orientation.
#
# UNITS:  Camera must be calibrated so that we can give feedback in known units.
# Distance units are inches
# Angle units are radians
def getTargetPositionEstimate():
    # TODO:  Actually do image processing...
    global dummy
    x = dummy
    y = dummy + 1.0
    theta = dummy + 2.0
    dummy += 3.0
    time.sleep(0.1)# Remove this, too - this simulates time to process image
    
    return x, y, theta

def main():
    # TODO:  Remove this global
    global dummy
    dummy = 0.0

    #roboRioIP = '10.31.67.2'
    roboRioIP = '127.0.0.1'
    roboRioPort = 5801# must be in the range 5800-5810, and must match the settings in the java robot code
    
    roboRioInterface = networkInterface(roboRioIP, roboRioPort)
    
    while True:
        # We'll just loop as fast as possible
        robotPosition = getTargetPositionEstimate()
        roboRioInterface.sendPosition(robotPosition)

if __name__ == '__main__':
    main()

        