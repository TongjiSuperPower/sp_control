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

class Contour:
    def __init__(self,centerpoint,obj) -> None:
        self.centerpoint = centerpoint
        self.obj = obj

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
# Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
config = stereo.initialConfig.get()
config.postProcessing.speckleFilter.enable = False
config.postProcessing.speckleFilter.speckleRange = 50
config.postProcessing.temporalFilter.enable = True
config.postProcessing.spatialFilter.enable = True
config.postProcessing.spatialFilter.holeFillingRadius = 2
config.postProcessing.spatialFilter.numIterations = 1
config.postProcessing.thresholdFilter.minRange = 400
config.postProcessing.thresholdFilter.maxRange = 30000
config.postProcessing.decimationFilter.decimationFactor = 1
stereo.initialConfig.set(config)
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


    ctrl = dai.CameraControl()
    ctrl.setManualExposure(2000, sensIso)
    controlQueue.send(ctrl)

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
            cv2.imshow(depthWindowName, frameDisp)

        if latestPacket["dep"] is not None:
            framedepth = latestPacket["dep"].getFrame()
            cv2.imshow("depth",framedepth)

        # Blend when both received
        if frameRgb is not None and frameDisp is not None and framedepth is not None:
            # Need to have both frames in BGR format before blending
            if len(frameDisp.shape) < 3:
                frameDisp = cv2.cvtColor(frameDisp, cv2.COLOR_GRAY2BGR)

            frame = frameRgb
            # blue, _, red = cv2.split(frame)
            # subtracted = cv2.subtract(red, blue)
            subtracted = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            # subtracted = cv2.subtract(blue, red)
            # subtracted = cv2.subtract(red,blue)
            _, threshed = cv2.threshold(subtracted, 120, 255, cv2.THRESH_BINARY)
            kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
            cv2.erode(threshed, kernal, dst=threshed)
            cv2.dilate(threshed, kernal, dst=threshed)
            cv2.imshow("hsv_image",threshed)
            cv2.imshow('b',frame) 
            contours, _ = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            cv2.drawContours(frame,contours,-1,(0, 255, 255),thickness = 2)
            contours_new = []
            point_array = []
            for contour in contours:
                if  500 < cv2.contourArea(contour) < 5000 :
                    rect = cv2.minAreaRect(contour)
                    center = rect[0]
                    h, w = rect[1]
                    if h < w:
                        h, w = w, h
                    ratio = h / w
                    if 0.25 < ratio < 5 :
                       contours_new.append(contour)
            # cv2.drawContours(frame,contours_new,-1,(0, 255, 255),thickness = 2)
            print(len(contours_new))

            quads = [] #array of quad including four peak points{}
            hulls = []
            for i in range(len(contours_new)):
                rect = cv2.minAreaRect(contours_new[i])
                center = rect[0]
                hull = cv2.convexHull(contours_new[i])
                # 阈值越小，越容易逼近
                quad = cv2.approxPolyDP(hull, 3, True)# maximum_area_inscribed
                # if 3 < len(quad) <6:
                #     print(quad)
                hulls.append(hull)
                quads.append(quad)
                point_array.append(center)
            # obj = np.array([[-112.5, 112.5, 0], [112.5, 112.5, 0], [-112.5, -112.5 , 0],[112.5, -112.5 ,0]
            #                 ],dtype=np.float64)
            cv2.drawContours(frame,quads,-1,(0, 255, 0),thickness = 2)
            print(point_array)
            if len(point_array) == 4:
                area_list = list(map(cv2.contourArea,quads))
                # print(area_list)
                id_min = area_list.index(min(area_list))
                # print(id_min)
                # print(point_array)
                origin = point_array[id_min]
                def clockwiseangle_and_distance(point):
                    refvec = [0,1]
                    # Vector between point and the origin: v = p - o
                    vector = [point[0]-origin[0], point[1]-origin[1]]
                    # Length of vector: ||v||
                    lenvector = math.hypot(vector[0], vector[1])
                    # If length is zero there is no angle
                    if lenvector == 0:
                        return -math.pi, 0
                    # Normalize vector: v/||v||
                    normalized = [vector[0]/lenvector, vector[1]/lenvector]
                    dotprod  = normalized[0]*refvec[0] + normalized[1]*refvec[1]     # x1*x2 + y1*y2
                    diffprod = refvec[1]*normalized[0] - refvec[0]*normalized[1]     # x1*y2 - y1*x2
                    angle = math.atan2(diffprod, dotprod)
                    # Negative angles represent counter-clockwise angles so we need to subtract them 
                    # from 2*pi (360 degrees)
                    if angle < 0:
                        return 2*math.pi+angle, lenvector
                    # I return first the angle because that's the primary sorting criterium
                    # but if two vectors have the same angle then the shorter distance should come first.
                    return angle, lenvector        
                point_array = sorted(point_array, key=clockwiseangle_and_distance)
                # print(point_array)
                point_rt = Contour(point_array[0],[112.5, 112.5, 0])
                point_lt = Contour(point_array[1],[-112.5, 112.5, 0])
                point_lb = Contour(point_array[2],[-112.5, -112.5, 0])
                point_rb = Contour(point_array[3],[112.5, -112.5, 0])
                    
                point_array = np.array(point_array,np.int32)
                # print(np.argsort(point_array[:,1]))
                # bbox = np.array(bbox,np.int32)
                cv2.polylines(frame, [point_array], True, (0, 255, 0), 2) 
                # cv2.polylines(frame, bbox, True, (255, 255, 0), 2)   
                
                
                obj = np.array([point_rt.obj,point_lt.obj,point_lb.obj,point_rb.obj
                                ],dtype=np.float64)
                point_array_n = point_array
                point_array_n = np.int32(point_array_n)
                pnts = np.array(point_array_n,dtype=np.float64) # 像素坐标
                success,rvec, tvec = cv2.solvePnP(obj, pnts, Camera_intrinsic["mtx"], Camera_intrinsic["dist"],flags=cv2.SOLVEPNP_ITERATIVE)
                rvec_matrix = cv2.Rodrigues(rvec)[0]
                # rvec_matrix = np.dot(R_gripper2base,R_camera2gimbal,rvec_matrix)
                r = R.from_matrix(rvec_matrix)
                Quaternion = r.as_quat()
                qx = Quaternion[0]
                qy = Quaternion[1]
                qz = Quaternion[2]
                qw = Quaternion[3]
        
                center_x = round((point_array_n[0][0] + point_array_n[1][0] +point_array_n[2][0]+point_array_n[3][0]) / 4)
                center_y = round((point_array_n[0][1] + point_array_n[1][1]+ point_array_n[2][1]+point_array_n[3][1]) / 4)
                roi = [center_x-10,center_y-10,center_x+10,center_y+10]
                # get 3D zuobiao
                spatials, centroid = hostSpatials.calc_spatials(framedepth, roi)
                cx = spatials['x']*0.001
                cy = spatials['y']*0.001
                cz = spatials['z']*0.001
                print(spatials)

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