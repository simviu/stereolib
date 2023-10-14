#!/usr/bin/env python3

from pathlib import Path
import cv2
import depthai as dai
from depthai_sdk import OakCamera
import time
import os
import threading

#----
class Frm(object):
    def __init__(self) -> None:
        self.left_rctf = None
        self.right_rctf = None
        self.color = None
        return
    #----
    def show(self):
        cv2.imshow("color",             self.color)
        cv2.imshow("Left Rectified",    self.left_rctf)
        cv2.imshow("Right Rectified",   self.right_rctf)
        cv2.waitKey(1)


#-----
class OakCamFeed(object):
    def __init__(self) -> None:
        self.frm_id_ = 0
        self.callbk_ = None #--- callbk_(f: Frm)
        self.cfg_en_wr_frms_ = False
        self.cfg_en_show_ = False
        self.running_ = False
        return
    
    #----
    def run(self):
        print("Init OakCamFeed...")
        self.frm_id_ = 0

        #--- TODO: has to be in main thread for Oak?
        self.run_thd_()

        #---- thread mode not working
        print("start Oak thread...")
        self.sync_thread_ = threading.Thread(target=self.run_thd_,  daemon=True)
        self.sync_thread_.start()
        print("Oak thread running.")

        return True
    
    #----    
    def run_thd_(self):
        print("  Building pipepline for Oak camera...")

        # Create pipeline
        pipeline = dai.Pipeline()

        # Define source and output
        camC = pipeline.create(dai.node.ColorCamera)
        camL = pipeline.create(dai.node.MonoCamera)
        camR = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        xC = pipeline.create(dai.node.XLinkOut)
        xL = pipeline.create(dai.node.XLinkOut)
        xR = pipeline.create(dai.node.XLinkOut)
        #xD = pipeline.create(dai.node.XLinkOut)
        #xN = pipeline.create(dai.node.XLinkOut)

        xC.setStreamName("color")
        xL.setStreamName("left_rctf")
        xR.setStreamName("right_rctf")
        #xD.setStreamName("depth")
        #xN.setStreamName("depthConf")

        # stereo settings
        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        # LR-check is required for depth alignment
        stereo.setLeftRightCheck(True)
        if 0: stereo.setSubpixel(True)  # TODO enable for test
        #stereo.setDepthAlign(dai.CameraBoardSocket.RGB)


        # Properties
        camC.setBoardSocket(dai.CameraBoardSocket.CENTER)
        camL.setBoardSocket(dai.CameraBoardSocket.LEFT)
        camR.setBoardSocket(dai.CameraBoardSocket.RIGHT)

#        camL.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)
#        camR.setResolution(dai.MonoCameraProperties.SensorResolution.THE_720_P)

        # Linking
        camC.isp.link(xC.input)
        camL.out.link(stereo.left)
        camR.out.link(stereo.right)
        stereo.rectifiedLeft.link(xL.input)# rectified
        stereo.rectifiedRight.link(xR.input)
        #stereo.depth.link(xD.input)
        #stereo.confidenceMap.link(xN.input)

        #---- create dirs
        for s in ["output", "output/frms"]:
            Path(s).mkdir(parents=True, exist_ok=True)

        for s in ["color",   "left_rctf", "right_rctf" ]:  
            Path("output/frms/"+s).mkdir(parents=True, exist_ok=True)

        # Connect to device and start pipeline
        with dai.Device(pipeline) as device:
            # Output queue will be used to get the grayscale frames from the output defined above

            qC = device.getOutputQueue(name="color", maxSize=4, blocking=False)
            qL = device.getOutputQueue(name="left_rctf", maxSize=4, blocking=False)
            qR = device.getOutputQueue(name="right_rctf", maxSize=4, blocking=False)
        #   qD = device.getOutputQueue(name="depth", maxSize=4, blocking=False)
        #   qN = device.getOutputQueue(name="depthConf", maxSize=4, blocking=False)
            print("  Oak camera start loop...")

            self.running_ = True
            #------
            while self.running_:
                self.get_frm_([qC, qL, qR])
    
    #-----------
    def get_frm_(self, ques):
        self.frm_id_ += 1
        fi = self.frm_id_

        # Blocking call, will wait until a new data has arrived 
        #        inR = qR.get()  
        qC, qL, qR = ques[0], ques[1], ques[2]
        inC = qC.get()
        inL = qL.get()
        inR = qR.get()
#        inD = qD.get()
#        inN = qN.get()
        # Data is originally represented as a flat 1D array, it needs to be converted into HxW form
        # Frame is transformed and ready to be shown
        imC = inC.getCvFrame()
        imL = inL.getCvFrame()
        imR = inR.getCvFrame()

        # to color mode
        imL = cv2.cvtColor(imL, cv2.COLOR_GRAY2BGR)
        imR = cv2.cvtColor(imR, cv2.COLOR_GRAY2BGR)

#       imD = inD.getCvFrame()
#       imN = inN.getCvFrame()

        #---- call back
        if self.callbk_ is not None:
            frm = Frm()

            frm.left_rctf = imL
            frm.right_rctf = imR
            frm.color = imC
            self.callbk_(frm)

        #-------------
        if self.cfg_en_show_:
            cv2.imshow("color", imC)
            cv2.imshow("Left Rectified",    imL)
            cv2.imshow("Right Rectified",   imR)

            
            #cv2.imshow("Depth", imD * 100)
            #cv2.imshow("N", imN)
        if cv2.waitKey(1) == ord('q'):
            self.running_ = False
            return
        #-------------
        # After showing the frame, it's being stored inside a target directory as a PNG image
        if self.cfg_en_wr_frms_:
            cv2.imwrite("output/frms/color/"+str(fi)+".png", imC)
            cv2.imwrite("output/frms/left_rctf/"+str(fi)+".png", imL)
            cv2.imwrite("output/frms/right_rctf/"+str(fi)+".png", imR)
#            cv2.imwrite("output/frms/depth/"+str(fi)+".png", imD)
#            cv2.imwrite("output/frms/depthConf/"+str(fi)+".png", imN)
            print("frm "+str(fi))

        #if cv2.waitKey(1) == ord('q'):
        #    break


        
#----------
# test
#----------
def test():
    camFd = OakCamFeed()
    camFd.callbk_ = lambda f : f.show()
    camFd.cfg_en_wr_frms_ = True
    #camFd.cfg_en_show_ = True
    camFd.run()

    while True:
        time.sleep(1)
        if cv2.waitKey(1) == ord('q'):
                break

    return False


#----------
# main
#----------
if __name__ == "__main__":
    test()
    