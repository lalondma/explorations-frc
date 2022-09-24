#!/usr/bin/env python3

# Copyright (c) FIRST and other WPILib contributors.
# Open Source Software; you can modify and/or share it under the terms of
# the WPILib BSD license file in the root directory of this project.

import json
import time
import sys
import numpy as np
import apriltag
import math
from cscore import CameraServer, VideoSource, UsbCamera, MjpegServer, CvSink
from networktables import NetworkTablesInstance
import cv2
#   JSON format:
#   {
#       "team": <team number>,
#       "ntmode": <"client" or "server", "client" if unspecified>
#       "cameras": [
#           {
#               "name": <camera name>
#               "path": <path, e.g. "/dev/video0">
#               "pixel format": <"MJPEG", "YUYV", etc>   // optional
#               "width": <video mode width>              // optional
#               "height": <video mode height>            // optional
#               "fps": <video mode fps>                  // optional
#               "brightness": <percentage brightness>    // optional
#               "white balance": <"auto", "hold", value> // optional
#               "exposure": <"auto", "hold", value>      // optional
#               "properties": [                          // optional
#                   {
#                       "name": <property name>
#                       "value": <property value>
#                   }
#               ],
#               "stream": {                              // optional
#                   "properties": [
#                       {
#                           "name": <stream property name>
#                           "value": <stream property value>
#                       }
#                   ]
#               }
#           }
#       ]
#       "switched cameras": [
#           {
#               "name": <virtual camera name>
#               "key": <network table key used for selection>
#               // if NT value is a string, it's treated as a name
#               // if NT value is a double, it's treated as an integer index
#           }
#       ]
#   }

configFile = "/boot/frc.json"

class CameraConfig: pass

team = None
server = False
cameraConfigs = []
switchedCameraConfigs = []
cameras = []

def parseError(str):
    """Report parse error."""
    print("config error in '" + configFile + "': " + str, file=sys.stderr)

def readCameraConfig(config):
    """Read single camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read camera name")
        return False

    # path
    try:
        cam.path = config["path"]
    except KeyError:
        parseError("camera '{}': could not read path".format(cam.name))
        return False

    # stream properties
    cam.streamConfig = config.get("stream")

    cam.config = config

    cameraConfigs.append(cam)
    return True

def readSwitchedCameraConfig(config):
    """Read single switched camera configuration."""
    cam = CameraConfig()

    # name
    try:
        cam.name = config["name"]
    except KeyError:
        parseError("could not read switched camera name")
        return False

    # path
    try:
        cam.key = config["key"]
    except KeyError:
        parseError("switched camera '{}': could not read key".format(cam.name))
        return False

    switchedCameraConfigs.append(cam)
    return True

def readConfig():
    """Read configuration file."""
    global team
    global server

    # parse file
    try:
        with open(configFile, "rt", encoding="utf-8") as f:
            j = json.load(f)
    except OSError as err:
        print("could not open '{}': {}".format(configFile, err), file=sys.stderr)
        return False

    # top level must be an object
    if not isinstance(j, dict):
        parseError("must be JSON object")
        return False

    # team number
    try:
        team = j["team"]
    except KeyError:
        parseError("could not read team number")
        return False

    # ntmode (optional)
    if "ntmode" in j:
        str = j["ntmode"]
        if str.lower() == "client":
            server = False
        elif str.lower() == "server":
            server = True
        else:
            parseError("could not understand ntmode value '{}'".format(str))

    # cameras
    try:
        cameras = j["cameras"]
    except KeyError:
        parseError("could not read cameras")
        return False
    for camera in cameras:
        if not readCameraConfig(camera):
            return False

    # switched cameras
    if "switched cameras" in j:
        for camera in j["switched cameras"]:
            if not readSwitchedCameraConfig(camera):
                return False

    return True

def startCamera(config):
    """Start running the camera."""
    print("Starting camera '{}' on {}".format(config.name, config.path))
    inst = CameraServer.getInstance()
    camera = UsbCamera(config.name, config.path)
    server = inst.startAutomaticCapture(camera=camera, return_server=True)

    camera.setConfigJson(json.dumps(config.config))
    camera.setConnectionStrategy(VideoSource.ConnectionStrategy.kKeepOpen)

    if config.streamConfig is not None:
        server.setConfigJson(json.dumps(config.streamConfig))

    return camera

def startSwitchedCamera(config):
    """Start running the switched camera."""
    print("Starting switched camera '{}' on {}".format(config.name, config.key))
    server = CameraServer.getInstance().addSwitchedCamera(config.name)

    def listener(fromobj, key, value, isNew):
        if isinstance(value, float):
            i = int(value)
            if i >= 0 and i < len(cameras):
              server.setSource(cameras[i])
        elif isinstance(value, str):
            for i in range(len(cameraConfigs)):
                if value == cameraConfigs[i].name:
                    server.setSource(cameras[i])
                    break

    NetworkTablesInstance.getDefault().getEntry(config.key).addListener(
        listener,
        NetworkTablesInstance.NotifyFlags.IMMEDIATE |
        NetworkTablesInstance.NotifyFlags.NEW |
        NetworkTablesInstance.NotifyFlags.UPDATE)

    return server

if __name__ == "__main__":
    if len(sys.argv) >= 2:
        configFile = sys.argv[1]
    print("!!")
    # read configuration
    if not readConfig():
        sys.exit(1)
    imW,imH = 640,480 #320,240
    # start NetworkTables
    ntinst = NetworkTablesInstance.getDefault()
    #ntinst.initialize(server="169.254.160.218")
    if server:
        print("Setting up NetworkTables server")
        ntinst.startServer()
    else:
        print("Setting up NetworkTables client for team {}".format(team))
        ntinst.startClient("169.254.160.218")
        ntinst.startDSClient()

    # start cameras
    for config in cameraConfigs:
        cameras.append(startCamera(config))

    # start switched cameras
    for config in switchedCameraConfigs:
        startSwitchedCamera(config)
    print("[INFO] detecting AprilTags...")
    options = apriltag.DetectorOptions(families="tag36h11")
    detector = apriltag.Detector(options)
    cameras[0].setResolution(imW,imH)
    img = np.zeros((imH,imW,3),dtype=np.uint8)
    if len(cameras)>0:
        cvsink = CvSink("sink") #CameraServer.getInstance().getVideo()
        cvsink.setSource(cameras[0])
     
    scale = 18
    objectPoints = np.array([[[-160, -160, 0], \
    [160, -160, 0], \
    [160, 160, 0], \
    [-160, 160, 0]]], \
    dtype=np.float32) / scale

    rVec = None
    tVec = None
    # loop forever
    flag=True
    iterate=False
    distCoeff = np.zeros((5,1))
    focal_length=517
    center=[322,245]
    camMatrix = np.array(
        [[focal_length, 0, center[0]],
        [0, focal_length, center[1]],
        [0, 0, 1]], dtype = "double"
        )
    #camMatrix = np.array(
    #    [[830, 0, 320],
    #    [0, 609, 240],
    #    [0, 0, 1]], dtype = "double"
    #    )
    #camMatrix = np.array( \
    ##[[1210.63920, 0, 320] \
    #[#0,1140.51499, 240] \
    #[0,0,1]], dtype = "double"
    #    )
    while True:
#        time.sleep(10)
#        sd=ntinst.getTable("SmartDashboard")
#        sd.putNumber('somemnumber',2)
#        print(ntinst.isConnected())
#        entry=ntinst.getTable("SmartDashboard").getEntry("detections")
#        entry.setString("{'x': 23, 'y':34}")

        t,imagergb = cvsink.grabFrame(img)
        gray=cv2.cvtColor(imagergb, cv2.COLOR_BGR2GRAY)
        results = detector.detect(gray)
        #print("[INFO] {} total AprilTags detected".format(len(results)))
        # loop over the AprilTag detection results
        for r in results:
           # extract the bounding box (x, y)-coordinates for the AprilTag
           # and convert each of the (x, y)-coordinate pairs to integers
           (ptA, ptB, ptC, ptD) = r.corners
           image=imagergb.copy()
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
           cv2.imwrite("z.png",image)
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
           
