
# first, import all necessary modules
from pathlib import Path

import blobconverter
import cv2
import depthai
import numpy as np

# Pipeline tells DepthAI what operations to perform when running - you define all of the resources used and flows here
pipeline = depthai.Pipeline()

with depthai.Device(pipeline) as device:
    # Print MxID, USB speed, and available cameras on the device
    print('MxId:',device.getDeviceInfo().getMxId())
    print('USB speed:',device.getUsbSpeed())
    print('Connected cameras:',device.getConnectedCameras())
