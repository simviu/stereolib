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

xC = pipeline.create(dai.node.XLinkOut)
xL = pipeline.create(dai.node.XLinkOut)
xR = pipeline.create(dai.node.XLinkOut)

xC.setStreamName("C")
xL.setStreamName("L")
xR.setStreamName("R")

# Properties
camC.setBoardSocket(dai.CameraBoardSocket.CENTER)
camL.setBoardSocket(dai.CameraBoardSocket.LEFT)
camR.setBoardSocket(dai.CameraBoardSocket.RIGHT)

# link
camC.isp.link(xC.input)
camL.out.link(xL.input)
camR.out.link(xR.input)

#---- create dirs
Path("frms").mkdir(parents=True, exist_ok=True)
Path("frms/C").mkdir(parents=True, exist_ok=True)
Path("frms/L").mkdir(parents=True, exist_ok=True)
Path("frms/R").mkdir(parents=True, exist_ok=True)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:
    # Output queue will be used to get the grayscale frames from the output defined above

    qC = device.getOutputQueue(name="C", maxSize=4, blocking=False)
    qL = device.getOutputQueue(name="L", maxSize=4, blocking=False)
    qR = device.getOutputQueue(name="R", maxSize=4, blocking=False)
 
    #------
    i=0
    while True:
        i = i + 1
        # Blocking call, will wait until a new data has arrived        inR = qR.get()  
        inC = qC.get()
        inL = qL.get()
        inR = qR.get()
        
        # Data is originally represented as a flat 1D array, it needs to be converted into HxW form
        # Frame is transformed and ready to be shown
        imC = inC.getCvFrame()
        imL = inL.getCvFrame()
        imR = inR.getCvFrame()
        
        cv2.imshow("C", imC)
        cv2.imshow("L", imL)
        cv2.imshow("R", imR)
        

        # After showing the frame, it's being stored inside a target directory as a PNG image
        cv2.imwrite("frms/C/"+str(i)+".png", imC)
        cv2.imwrite("frms/L/"+str(i)+".png", imL)
        cv2.imwrite("frms/R/"+str(i)+".png", imR)
        
        print("frm "+str(i))

        if cv2.waitKey(1) == ord('q'):
            break
