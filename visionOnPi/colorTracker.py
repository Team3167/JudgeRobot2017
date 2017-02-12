#! /usr/bin/env python 
import cv2
import numpy as np
import time
 
def processAFrame(capture):

    #get a frame from RGB camera
    successful, frame = capture.read()
    if not successful:
      #We failed to read an image from the capture feed. 
      # Return
      print("Failed to capture data")
      return

    # Convert BGR to HSV
    hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
    
    # define range of green color in HSV
    lower_green = np.array([30,0,0])
    upper_green = np.array([80,255,255])
    
    # Threshold the HSV image to get only green colors. This is used to find the contours
    mask = cv2.inRange(hsv, lower_green, upper_green)
    
    # Bitwise-AND mask and original image, only used for visualization
    res = cv2.bitwise_and(frame,frame, mask= mask)
    
    #Thresholding to find contours. Contours are 
    ret,thresh = cv2.threshold(mask,127,255,0)
    contours,hierarchy = cv2.findContours(thresh, 1, 2)

    #Get the two largest contours
    sortedContors = sorted(contours, key=cv2.contourArea)
    contour1 = sortedContors[-1]
    contour2 = sortedContors[-2]

    #This code is for the actual visualization component. 
    # This is useful for debugging but on the actual robot it should 
    # be deleted. 

    #To get everything to work properly you will have to play 
    # around with the masks above to figure out
    # what the right numbers you want are. 
    

    #Find and draw the rotated rectangles
    rect1 = cv2.minAreaRect(contour1)
    rect2 = cv2.minAreaRect(contour2)
    print("rect 1: " + str(rect1))
    print("rect 2: " + str(rect2))
    box1 = cv2.cv.BoxPoints(rect1)
    box1 = np.int0(box1)
    box2 = cv2.cv.BoxPoints(rect2)
    box2 = np.int0(box2)
    cv2.drawContours(res,[box1, box2],-1,(0,0,255),2)

    cv2.imshow("frame",res)
    cv2.waitKey(1)

def main():
    #Get a video capture object, first camera it can find
    cap = cv2.VideoCapture(0)
    for i in xrange(0, 100):
        processAFrame(cap)
        time.sleep(.1)
    cap.release()

if __name__ == '__main__':
    main()

        
        
         
        






