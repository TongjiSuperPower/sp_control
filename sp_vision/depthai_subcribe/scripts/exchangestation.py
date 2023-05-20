import depthai as dai
from calc import HostSpatialsCalc
import math
import numpy as np
import cv2
import time
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import rospy

# Step size ('W','A','S','D' controls)
STEP_SIZE = 8
# Manual exposure/focus/white-balance set step
EXP_STEP = 500  # us
ISO_STEP = 50
LENS_STEP = 3
WB_STEP = 200
expTime = 20000
sensIso = 800

def clamp(num, v0, v1):
    return max(v0, min(num, v1))


#qie ge huang se
params = {'hue_low': 8, 'saturation_low': 110, 'value_low': 0,
          'hue_high': 30, 'saturation_high': 255, 'value_high': 255,
          'min_area_size': 5000, 'max_area_size': 400000}
thresh_low = (params['hue_low'], params['saturation_low'], params['value_low'])
thresh_high = (params['hue_high'], params['saturation_high'], params['value_high'])
min_area_size = params['min_area_size']
max_area_size = params['max_area_size']
lower_silver = (120,50,100)
upper_silver = (150,255,255)
lower_black = np.array([0,0,0]) 
upper_black= np.array([180, 255, 43])
Camera_intrinsic = {

    "mtx": np.array([[1025.35323576971, 0, 678.7266096913569],
 [0, 1027.080747783651, 396.9785232682882],
 [0, 0, 1]],   dtype=np.double),
    "dist": np.array([[0.220473935504143, -1.294976334542626, 0.003407354582097702, -0.001096689035107743, 2.91864887650898]], dtype=np.double),

}

# Optional. If set (True), the ColorCamera is downscaled from 1080p to 720p.
# Otherwise (False), the aligned depth is automatically upscaled to 1080p
downscaleColor = True
fps = 15
# The disparity is computed at this resolution, then upscaled to RGB resolution
monoResolution = dai.MonoCameraProperties.SensorResolution.THE_720_P

# Create pipeline
pipeline = dai.Pipeline()
device = dai.Device()
queueNames = []

# Define sources and outputs
camRgb = pipeline.create(dai.node.ColorCamera)
left = pipeline.create(dai.node.MonoCamera)
right = pipeline.create(dai.node.MonoCamera)
stereo = pipeline.create(dai.node.StereoDepth)

rgbOut = pipeline.create(dai.node.XLinkOut)
disparityOut = pipeline.create(dai.node.XLinkOut)
depthout = pipeline.create(dai.node.XLinkOut)

rgbOut.setStreamName("rgb")
queueNames.append("rgb")
disparityOut.setStreamName("disp")
queueNames.append("disp")
depthout.setStreamName("dep")
queueNames.append("dep")

#Properties
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
camRgb.setFps(fps)
if downscaleColor: camRgb.setIspScale(2, 3)
# For now, RGB needs fixed focus to properly align with depth.
# This value was used during calibration
try:
    calibData = device.readCalibration2()
    lensPosition = calibData.getLensPosition(dai.CameraBoardSocket.RGB)
    if lensPosition:
        camRgb.initialControl.setManualFocus(lensPosition)
except:
    raise
left.setResolution(monoResolution)
left.setBoardSocket(dai.CameraBoardSocket.LEFT)
left.setFps(fps)
right.setResolution(monoResolution)
right.setBoardSocket(dai.CameraBoardSocket.RIGHT)
right.setFps(fps)

stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# LR-check is required for depth alignment
stereo.setLeftRightCheck(True)
stereo.setDepthAlign(dai.CameraBoardSocket.RGB)
stereo.setExtendedDisparity(True)
# Linking
camRgb.isp.link(rgbOut.input)
left.out.link(stereo.left)
right.out.link(stereo.right)
stereo.disparity.link(disparityOut.input)
stereo.depth.link(depthout.input)

controlIn = pipeline.create(dai.node.XLinkIn)
controlIn.setStreamName('control')
controlIn.out.link(camRgb.inputControl)

hostSpatials = HostSpatialsCalc(device)
delta = 5
hostSpatials.setDeltaRoi(delta)

# Connect to device and start pipeline
with device:
    device.startPipeline(pipeline)
    controlQueue = device.getInputQueue('control')

    frameRgb = None
    frameDisp = None
    framedepth = None

    # Configure windows; trackbar adjusts blending ratio of rgb/depth
    rgbWindowName = "rgb"
    depthWindowName = "depth"
    blendedWindowName = "rgb-depth"
    cv2.namedWindow(rgbWindowName)
    cv2.namedWindow(depthWindowName)
    cv2.namedWindow(blendedWindowName)
    
    while True:
        latestPacket = {}
        latestPacket["rgb"] = None
        latestPacket["disp"] = None
        latestPacket["dep"] = None

        queueEvents = device.getQueueEvents(("rgb", "disp","dep"))
        for queueName in queueEvents:
            packets = device.getOutputQueue(queueName).tryGetAll()
            if len(packets) > 0:
                latestPacket[queueName] = packets[-1]

        if latestPacket["rgb"] is not None:
            frameRgb = latestPacket["rgb"].getCvFrame()
            cv2.imshow(rgbWindowName, frameRgb)
        

        if latestPacket["disp"] is not None:
            frameDisp = latestPacket["disp"].getFrame()
            maxDisparity = stereo.initialConfig.getMaxDisparity()
            # Optional, extend range 0..95 -> 0..255, for a better visualisation
            if 1: frameDisp = (frameDisp * 255. / maxDisparity).astype(np.uint8)
            # Optional, apply false colorization
            # if 1: frameDisp = cv2.applyColorMap(frameDisp, cv2.COLORMAP_HOT)
            frameDisp = np.ascontiguousarray(frameDisp)
            # cv2.imshow(depthWindowName, frameDisp)

        if latestPacket["dep"] is not None:
            framedepth = latestPacket["dep"].getFrame()
            # cv2.imshow("depth",framedepth)

        # Blend when both received
        if frameRgb is not None and frameDisp is not None and framedepth is not None:
            # Need to have both frames in BGR format before blending
            if len(frameDisp.shape) < 3:
                frameDisp = cv2.cvtColor(frameDisp, cv2.COLOR_GRAY2BGR)

            frame = frameRgb
            blue, _, red = cv2.split(frame)
            # subtracted = cv2.subtract(red, blue)
            # subtracted = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            subtracted = cv2.subtract(blue, red)
            # subtracted = cv2.subtract(red,blue)
            _, threshed = cv2.threshold(subtracted, 100, 255, cv2.THRESH_BINARY)
            kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
            cv2.erode(threshed, kernal, dst=threshed)
            cv2.dilate(threshed, kernal, dst=threshed)
            cv2.imshow("hsv_image",threshed)
            cv2.imshow('b',frame) 
        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key in [ord('i'), ord('o'), ord('k'), ord('l')]:
            if key == ord('i'):
                expTime -= EXP_STEP
            if key == ord('o'):
                expTime += EXP_STEP
            if key == ord('k'):
                sensIso -= ISO_STEP
            if key == ord('l'):
                sensIso += ISO_STEP
            expTime = clamp(expTime, 1, 33000)
            sensIso = clamp(sensIso, 100, 1600)
            print("Setting manual exposure, time: ", expTime, "iso: ", sensIso)
            ctrl = dai.CameraControl()
            ctrl.setManualExposure(expTime, sensIso)
            controlQueue.send(ctrl)