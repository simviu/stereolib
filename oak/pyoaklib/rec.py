from depthai_sdk import OakCamera, RecordType
import depthai
import os

oak = OakCamera(usbSpeed=depthai.UsbSpeed.HIGH)
if oak is None:
    print("Failed to init OakCamera()")
    os.exit(1)

# Create color camera
camc = oak.create_camera('color')
caml = oak.create_camera('left')
camr = oak.create_camera('right')

# Visualize color camera frame stream
oak.visualize(camc, fps=True, record="camc.mp4")
oak.visualize(caml, fps=True, record="caml.mp4")
oak.visualize(camr, fps=True, record="camr.mp4")
# Start the pipeline, continuously poll
oak.start(blocking=True)
