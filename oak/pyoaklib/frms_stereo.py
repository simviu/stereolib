#!/usr/bin/env python3

from pathlib import Path
import cv2
import depthai as dai
from depthai_sdk import OakCamera
import time
import os



# Create pipeline
pipeline = dai.Pipeline()


# Define source and output
camC = pipeline.create(dai.node.ColorCamera)
camL = pipeline.create(dai.node.MonoCamera)
camR = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

xC = pipeline.create(dai.node.XLinkOut)
xD = pipeline.create(dai.node.XLinkOut)
xN = pipeline.create(dai.node.XLinkOut)
xL = pipeline.create(dai.node.XLinkOut)
xR = pipeline.create(dai.node.XLinkOut)

xC.setStreamName("color")
xD.setStreamName("depth")
xN.setStreamName("depthConf")
xL.setStreamName("left")
xR.setStreamName("right")

# stereo settings
stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# LR-check is required for depth alignment
stereo.setLeftRightCheck(True)
if 0: stereo.setSubpixel(True)  # TODO enable for test
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)


# Properties
camC.setBoardSocket(dai.CameraBoardSocket.CENTER)
camL.setBoardSocket(dai.CameraBoardSocket.LEFT)
camR.setBoardSocket(dai.CameraBoardSocket.RIGHT)

camL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
camR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

# Linking

# Linking
camC.isp.link(xC.input)
camL.out.link(stereo.left)
camR.out.link(stereo.right)
camL.out.link(xL.input)
camR.out.link(xR.input)
stereo.depth.link(xD.input)
stereo.confidenceMap.link(xN.input)

#---- create dirs
Path("frms").mkdir(parents=True, exist_ok=True)
Path("frms/color").mkdir(parents=True, exist_ok=True)
Path("frms/depth").mkdir(parents=True, exist_ok=True)
Path("frms/depthConf").mkdir(parents=True, exist_ok=True)
Path("frms/left").mkdir(parents=True, exist_ok=True)
Path("frms/right").mkdir(parents=True, exist_ok=True)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    # Output queue will be used to get the grayscale frames from the output defined above

    qC = device.getOutputQueue(name="color", maxSize=4, blocking=False)
    qD = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
    qN = device.getOutputQueue(name="depthConf", maxSize=4, blocking=False)
    qL = device.getOutputQueue(name="left", maxSize=4, blocking=False)
    qR = device.getOutputQueue(name="right", maxSize=4, blocking=False)

    #------
    i=0
    while True:
        i = i + 1
        # Blocking call, will wait until a new data has arrived        inR = qR.get()  
        inC = qC.get()
        inD = qD.get()
        inN = qN.get()
        inL = qL.get()
        inR = qR.get()
        # Data is originally represented as a flat 1D array, it needs to be converted into HxW form
        # Frame is transformed and ready to be shown
        imC = inC.getCvFrame()
        imD = inD.getCvFrame()
        imN = inN.getCvFrame()
        imL = inL.getCvFrame()
        imR = inR.getCvFrame()
        cv2.imshow("color", imC)
        #cv2.imshow("D", imD)
        #cv2.imshow("N", imN)
        #cv2.imshow("L", imD)
        #cv2.imshow("R", imN)

        # After showing the frame, it's being stored inside a target directory as a PNG image
        cv2.imwrite("frms/color/"+str(i)+".png", imC)
        cv2.imwrite("frms/depth/"+str(i)+".png", imD)
        cv2.imwrite("frms/depthConf/"+str(i)+".png", imN)
        cv2.imwrite("frms/left/"+str(i)+".png", imL)
        cv2.imwrite("frms/right/"+str(i)+".png", imR)
        print("frm "+str(i))

        if cv2.waitKey(1) == ord('q'):
            break
