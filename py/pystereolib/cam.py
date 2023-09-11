
from pathlib import Path
import cv2
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
