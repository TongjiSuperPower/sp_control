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
    def __init__(self,centerpoint,obj,contour = 0) -> None:
        self.centerpoint = centerpoint
        self.couter = contour
        self.obj = obj


def find_parallel_quadrilateral(points):
    points = np.array(points)
    pointlist = []
    # 计算点对之间的距离
    distances = []
    for i in range(len(points)):
        for j in range(i+1, len(points)):
            distance = np.linalg.norm(points[i] - points[j])
            distances.append((i, j, distance))
    # 根据距离进行排序
    distances.sort(key=lambda x: x[2])
    
    # 寻找可以组成平行四边形的四个点
    for i in range(len(distances)-3):
        # 获取当前距离对的索引
        index1, index2, distance1 = distances[i]
        
        for j in range(i+1, len(distances)-2):
            # 获取下一个距离对的索引
            index3, index4, distance2 = distances[j]
            
            if index1 != index3 and index2 != index3 and index1 != index4 and index2 != index4: 
                # 检查是否满足平行四边形的几何关系
                if 0.85 < distance1/distance2 < 1.2:
                    # 获取四个点的坐标
                    point1 = points[index1]
                    point2 = points[index2]
                    point3 = points[index3]
                    point4 = points[index4]
                    
                    # 检查是否满足平行四边形的几何关系
                    if 0.85 < np.linalg.norm(point1 - point3) / np.linalg.norm(point2 - point4) < 1.2:
                        # 返回找到的四个点
                        pointlist.append([point1, point2, point3, point4, distance1/np.linalg.norm(point1 - point3)])
                        
    if len(pointlist) == 0:
    # 如果未找到满足条件的四个点，则返回None
        return None
    else:
        min = 1 
        a = 0
        for i in range(len(pointlist)):
            if min > abs(1 - pointlist[i][4]):
                min = abs(1 - pointlist[i][4])
                a = i
        return pointlist[a][0:4]

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
            # cv2.imshow(rgbWindowName, frameRgb)
        

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
            subtracted = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            _, threshed = cv2.threshold(subtracted, 50, 255, cv2.THRESH_BINARY)
            kernal = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
            cv2.erode(threshed, kernal, dst=threshed)
            cv2.dilate(threshed, kernal, dst=threshed)
            cv2.imshow('b',threshed) 
            contours, _ = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            # cv2.drawContours(frame,contours,-1,(0, 255, 255),thickness = 2)
            contours_new = []
            point_array = []
            quads = [] #array of quad including four peak points{}
            distance = []

            for contour in contours:
                if  700 < cv2.contourArea(contour) < 7000 :
                    
                    rect = cv2.minAreaRect(contour)
                    center = rect[0]
                    h, w = rect[1]
                    if h < w:
                        h, w = w, h
                    ratio = h / w
                    if 0.2 < ratio < 5 :
                        hull = cv2.convexHull(contour)
                        # 阈值越小，越容易逼近
                        quad = cv2.approxPolyDP(hull, 7, True)# maximum_area_inscribed
                        if len(quad) >= 4:
                            spatials, centroid = hostSpatials.calc_spatials(framedepth, [int(center[0]-10),int(center[1]-10),int(center[0]+10),int(center[1]+10)])
                            if spatials['z'] < 1000:
                                quads.append(quad)
                                point_array.append(center)
                                
            # obj = np.array([[-112.5, 112.5, 0], [112.5, 112.5, 0], [-112.5, -112.5 , 0],[112.5, -112.5 ,0]
            #                 ],dtype=np.float64)
            cv2.drawContours(frame,quads,-1,(0, 255, 0),thickness = 2)
            cv2.imshow("a",frame)
            
            four_point = find_parallel_quadrilateral(point_array)
            # print(four_point)
            fourcontour = []
            point_array = np.array(point_array)
            print(point_array)
            if four_point is not None:
                for point in four_point:
                    indices = np.where((point_array[:, 0] == point[0]) & (point_array[:, 1] == point[1]))
                    print(int(indices[0]))
                    fourcontour.append(quads[int(indices[0])])
                    
                area_list = list(map(cv2.contourArea,fourcontour))
                id_min = area_list.index(min(area_list))
                origin = four_point[id_min]
                center_x = round((four_point[0][0] + four_point[1][0] + four_point[2][0] + four_point[3][0]) / 4)
                center_y = round((four_point[0][1] + four_point[1][1] + four_point[2][1] + four_point[3][1]) / 4)
                
                def clockwiseangle_and_distance(point):
                    vect = [(point[0] - center_x),(point[1] - center_y)]
                    angle_rad = np.arctan2(vect[1], vect[0])
                    angle_deg = np.degrees(angle_rad)
                    return angle_deg      
                point_array = sorted(four_point,key=clockwiseangle_and_distance)
                print(point_array)
                # 查找目标点的索引
                indices = np.where(np.all(point_array == np.array(origin), axis=1))[0]
                # indices = np.where((point_array[:, 0] == origin[0]) & (point_array[:, 1] == origin[1]))
                # print(indices)

                point_array = np.roll(point_array,4 - int(indices))
                print('a' , point_array)
                point_rt = Contour(point_array[0],[112.5, 112.5, 0])
                point_lt = Contour(point_array[1],[-112.5, 112.5, 0])
                point_lb = Contour(point_array[2],[-112.5, -112.5, 0])
                point_rb = Contour(point_array[3],[112.5, -112.5, 0])                    
                point_array = np.array(point_array,np.int32)
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