#!/usr/bin/env python3

import time
import numpy as np
import pupil_apriltags as apriltag
import math
import cv2


if __name__ == "__main__":

    imW,imH = 640,480 

    print("[INFO] detecting AprilTags...")
    #options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(families="tag36h11")
    
    camera=cv2.VideoCapture(0)
    if not camera.isOpened():
        print("No camera!")
        exit(1)
        
    img = np.zeros((imH, imW, 3),dtype=np.uint8)
     
    # Paramètres de caméra
    scale = 18
    objectPoints = np.array([[[-160, -160, 0], \
    [160, -160, 0], \
    [160, 160, 0], \
    [-160, 160, 0]]], \
    dtype=np.float32) / scale
    
    rVec = None
    tVec = None
    
    # loop forever
    flag = True
    iterate = False
    distCoeff = np.array([-0.5, 3.3, 0.03, -0.05, -5.7])
    focal_length = 600
    center = [320, 240]
    camMatrix = np.array(
        [[focal_length, 0, center[0]],
        [0, focal_length, center[1]],
        [0, 0, 1]], dtype = "double")
        
    # Calcul de fps
    start_time = time.time()
    counter = 0
    freq = 1

    while True:
        
        t,imagergb = camera.read()
        gray = cv2.cvtColor(imagergb, cv2.COLOR_BGR2GRAY)
        # Détection du tag
        results = detector.detect(gray)
        
        # loop over the AprilTag detection results
        for r in results:
           # extract the bounding box (x, y)-coordinates for the AprilTag
           # and convert each of the (x, y)-coordinate pairs to integers
           (ptA, ptB, ptC, ptD) = r.corners
           image = imagergb.copy()
           ptB = (int(ptB[0]), int(ptB[1]))
           ptC = (int(ptC[0]), int(ptC[1]))
           ptD = (int(ptD[0]), int(ptD[1]))
           ptA = (int(ptA[0]), int(ptA[1]))
           # draw the bounding box of the AprilTag detection
           cv2.line(image, ptA, ptB, (0, 255, 0), 2)
           cv2.line(image, ptB, ptC, (0, 255, 0), 2)
           cv2.line(image, ptC, ptD, (0, 255, 0), 2)
           cv2.line(image, ptD, ptA, (0, 255, 0), 2)
           # draw the center (x, y)-coordinates of the AprilTag
           (cX, cY) = (int(r.center[0]), int(r.center[1]))
           cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
           tagFamily = r.tag_family.decode("utf-8")
           print("[INFO] tag family: {}".format(tagFamily))
           image = cv2.resize(image, (320,240))
           cv2.imshow("s", image)
           
           # April Tags have sub-pixel accuracy already
           imagePoints = r.corners.reshape(1,4,2)
        
           good, rVec, tVec = cv2.solvePnP(objectPoints, imagePoints, \
             camMatrix, distCoeff, rVec, tVec, iterate, cv2.SOLVEPNP_ITERATIVE)

           if good:
              iterate = True
              print("tVec: %.3f, %.3f, %.3f" % (tVec[0][0], tVec[1][0], tVec[2][0]))              

            # Can not be 'behind' barcode, or too far away
           if tVec[2][0] < 0 or tVec[2][0] > 1000:
                rVec = None
                tVec = None
                iterate = False
                continue

           dst, jacobian = cv2.Rodrigues(rVec)
           x = tVec[0][0]
           y = tVec[2][0]
           t = (math.asin(-dst[0][2]))

           Rx = y * (math.cos((math.pi/2) - t))
           Ry = y * (math.sin((math.pi/2) - t))

           print("Rx, Ry: %.3f, %.3f" % (Rx, Ry))
           
        counter += 1
        cv2.waitKey(1)
        if (time.time()-start_time) > freq:
          print("FPS: ", counter/(time.time()-start_time))
          counter = 0
          start_time = time.time()   
