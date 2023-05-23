import depthai as dai
from calc import HostSpatialsCalc
import math
import numpy as np
import cv2
import time
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import rospy

R_camera2gimbal = np.float32([[0.41016693024568496, 0.8763608546753984, -0.2524970133000318], [-0.909749552097839, 0.41263930117435166, -0.04565697738707521], [0.06417820339190353, 0.24843602701489942, 0.9665198904784318]])
t_camera2gimbal = np.float32([[-18.959828191075893], [1.5586220537514408], [-1.4427664518406154]])
R_gripper2base = 0
T_gripper2base = 0
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

def find_golden_ore(color_image):
    # 高斯滤波
    cv2.GaussianBlur(color_image, (5, 5), 3, dst=color_image)
    # 把BGR通道转化为HSV色彩空间
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    # 识别矿石的颜色特征
    ore_image = cv2.inRange(hsv_image, thresh_low, thresh_high)
    # 对识别后的图像进行腐蚀与膨胀，消除较小的连通域
    kernal = cv2.getStructuringElement(0, (3, 3))
    cv2.erode(ore_image, kernal, dst=ore_image)
    cv2.dilate(ore_image, kernal, dst=ore_image)
	# 轮廓识别
    contours, hierarchy = cv2.findContours(ore_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        # 计算所有轮廓的面积
        area_size = list(map(cv2.contourArea, contours))
        # 取最大面积
        max_size = max(area_size)
        max_area_index = area_size.index(max_size)
        # 若面积在阈值范围内，则认为识别到了矿石
        if min_area_size < max_size < max_area_size:
            box = cv2.boundingRect(contours[max_area_index])
            return box
    return None

def find_silver_ore(color_image):
    # 高斯滤波
    cv2.GaussianBlur(color_image, (5, 5), 3, dst=color_image)
    # 把BGR通道转化为HSV色彩空间
    hsv_image = cv2.cvtColor(color_image, cv2.COLOR_BGR2HSV)
    # 识别矿石的颜色特征
    ore_image = cv2.inRange(hsv_image, thresh_low, thresh_high)
    # 对识别后的图像进行腐蚀与膨胀，消除较小的连通域
    kernal = cv2.getStructuringElement(0, (3, 3))
    cv2.erode(ore_image, kernal, dst=ore_image)
    cv2.dilate(ore_image, kernal, dst=ore_image)
	# 轮廓识别
    contours, hierarchy = cv2.findContours(ore_image, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)
    if len(contours) > 0:
        # 计算所有轮廓的面积
        area_size = list(map(cv2.contourArea, contours))
        # 取最大面积
        max_size = max(area_size)
        max_area_index = area_size.index(max_size)
        # 若面积在阈值范围内，则认为识别到了矿石
        if min_area_size < max_size < max_area_size:
            box = cv2.boundingRect(contours[max_area_index])
            return box
    return None


def EulerAndQuaternionTransform( intput_data):
    """
        四元素与欧拉角互换
    """
    data_len = len(intput_data)
    angle_is_not_rad = False
 
    if data_len == 3:
        r = 0
        p = 0
        y = 0
        if angle_is_not_rad: # 180 ->pi
            r = math.radians(intput_data[0]) 
            p = math.radians(intput_data[1])
            y = math.radians(intput_data[2])
        else:
            r = intput_data[0] 
            p = intput_data[1]
            y = intput_data[2]
 
        sinp = math.sin(p/2)
        siny = math.sin(y/2)
        sinr = math.sin(r/2)
 
        cosp = math.cos(p/2)
        cosy = math.cos(y/2)
        cosr = math.cos(r/2)
 
        w = cosr*cosp*cosy + sinr*sinp*siny
        x = sinr*cosp*cosy - cosr*sinp*siny
        y = cosr*sinp*cosy + sinr*cosp*siny
        z = cosr*cosp*siny - sinr*sinp*cosy
        return [w,x,y,z]
 
    elif data_len == 4:
 
        w = intput_data[0] 
        x = intput_data[1]
        y = intput_data[2]
        z = intput_data[3]
 
        r = math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
        p = math.asin(2 * (w * y - z * x))
        y = math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
 
        if angle_is_not_rad : # pi -> 180
            r = math.degrees(r)
            p = math.degrees(p)
            y = math.degrees(y)
        return [r,p,y]

#已知平行四边形三个点，求第四个点
#计算两点之间的距离
def CalcEuclideanDistance(point1,point2):
    vec1 = np.array(point1)
    vec2 = np.array(point2)
    distance = np.linalg.norm(vec1 - vec2)
    return distance
#计算第四个点
def CalcFourthPoint(point1,point2,point3): #pint3为A点
    D = (point1[0]+point2[0]-point3[0],point1[1]+point2[1]-point3[1])
    return D
#三点构成一个三角形，利用两点之间的距离，判断邻边AB和AC,利用向量法以及平行四边形法则，可以求得第四个点D
def JudgeBeveling(point1,point2,point3):
    dist1 = CalcEuclideanDistance(point1,point2)
    dist2 = CalcEuclideanDistance(point1,point3)
    dist3 = CalcEuclideanDistance(point2,point3)
    dist = [dist1, dist2, dist3]
    max_dist = dist.index(max(dist))
    if max_dist == 0:
        D = CalcFourthPoint(point1,point2,point3)
    elif max_dist == 1:
        D = CalcFourthPoint(point1,point3,point2)
    else:
        D = CalcFourthPoint(point2,point3,point1)
    return D

def getpose(pose):
    global R_gripper2base,T_gripper2base
    Gripperq=[pose.orientation.w, pose.orientation.x ,pose.orientation.y, pose.orientation.z]
    Grippert=[pose.position.x,pose.position.y,pose.position.z]
    Gripperq = np.array(Gripperq)
    Grippert = np.array(Grippert)
    Rm = R.from_quat(Gripperq)
    # R_gripper2base = Rm.as_matrix()
    print("successfully")
    R_gripper2base = Rm.as_matrix()
    print(R_gripper2base)
    # T_gripper2base = Grippert
    T_gripper2base = Grippert
    # sub.unregister() 

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
# Create a node that will produce the depth map (using disparity output as it's easier to visualize depth this way)
# stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
# # Options: MEDIAN_OFF, KERNEL_3x3, KERNEL_5x5, KERNEL_7x7 (default)
# stereo.initialConfig.setMedianFilter(dai.MedianFilter.KERNEL_7x7)
# config = stereo.initialConfig.get()
# config.postProcessing.speckleFilter.enable = False
# config.postProcessing.speckleFilter.speckleRange = 50
# config.postProcessing.temporalFilter.enable = True
# config.postProcessing.spatialFilter.enable = True
# config.postProcessing.spatialFilter.holeFillingRadius = 2
# config.postProcessing.spatialFilter.numIterations = 1
# config.postProcessing.thresholdFilter.minRange = 400
# config.postProcessing.thresholdFilter.maxRange = 15000
# config.postProcessing.decimationFilter.decimationFactor = 1
# stereo.initialConfig.set(config)

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
    ctrl.setManualExposure(8000, sensIso)
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
            frame1 = frameRgb
            # cv2.imshow("a",frameRgb)
            ore_golden_box = find_golden_ore(frameRgb)
            ore_silver_box = find_silver_ore(frame1)
            if ore_golden_box != None:
                ore_box = ore_golden_box
            else:
                ore_box = ore_silver_box
            # ore_box = 1

            if ore_box != None:

                # rospy.init_node("cali_listener")
                # sub = rospy.Subscriber("calibrate",Pose,getpose,queue_size=10)
                # print("a")
                # rospy.spin() 

                hsv_image = cv2.GaussianBlur(frame, (5, 5), 0)

                # # 颜色空间转换，将RGB图像转换成HSV图像
                # hsv_image = cv2.cvtColor(hsv_image, cv2.COLOR_BGR2HSV)
                subtracted = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
                threshed = cv2.adaptiveThreshold(subtracted,255,cv2.ADAPTIVE_THRESH_GAUSSIAN_C,0,11,2)
                
                # # 根据阈值，去除背景
                mask = cv2.inRange(hsv_image, lower_black, upper_black)
                # # 对识别后的图像进行腐蚀与膨胀，消除较小的连通域 交叉型 7x7 效果较好
                kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (7, 7))
                # 先膨胀后腐蚀
                cv2.dilate(mask, kernal, dst=mask)
                cv2.erode(mask, kernal, dst=mask)
                cv2.imshow('a',mask)
                contours, hierarchy = cv2.findContours(mask, cv2.RETR_CCOMP, cv2.CHAIN_APPROX_SIMPLE)
                contours_new = []
                width_array = []
                height_array = []
                point_array = []
                angle_array = []
                if len(contours) > 0:
                    for cont in contours:
                        rect = cv2.minAreaRect(cont)
                        width = rect[1][0]
                        height = rect[1][1]
                        area = cv2.contourArea(cont)
                        if width > height:
                            width,height = height,width
                        #先交换长宽
                        if width != 0:
                            # 面积大小会随分辨率的改变而改变，注意在固定分辨率下进行调参
                            # if height < 200 and width < 200 and height/width < 5  and 1500 < area < 5000 :                   
                            if  height/width < 2.5 and 2000 < area < 30000 :                   
                                width_array.append(width)
                                height_array.append(height)
                                point_array.append(rect[0])
                                contours_new.append(cont)
                                angle_array.append(rect[2])
                                
                    # framecopy = np.copy(self.cv_image)
                    # 用深度会是部分角轮廓丢失
                    width_array_n=[]
                    height_array_n=[]
                    point_array_n=[]
                    contours_new_n=[] 
                    q = []
                    for i in range(len(contours_new)):
                        # print(len(contours_new[i]))  
                        # 不知道为什么在方形轮廓点数为50+ ，可能要多边形逼近 
                        hull = cv2.convexHull(contours_new[i])
                        quad = cv2.approxPolyDP(hull, 3, True)                           
                        if 3 < len(quad) <6:
                            if ore_box[0] < point_array[i][0] < ore_box[0]+ore_box[2] and ore_box[1] < point_array[i][1] < ore_box[1]+ore_box[3]:
                                contours_new_n.append(contours_new[i])
                                point_array_n.append(point_array[i]) 
                                width_array_n.append(width_array[i])
                                height_array_n.append(height_array[i])
                                q.append(quad)

                    quads = [] #array of quad including four peak points
                    hulls = []
                    suc = True
                    for i in range(len(contours_new_n)):
                        if (hierarchy[0, i, 3] < 0 and contours_new_n[i].shape[0] >= 4):
                            area = cv2.contourArea(contours_new_n[i])
                            if 2000 < area < 8000 :
                                hull = cv2.convexHull(contours_new_n[i])
                                if (area / cv2.contourArea(hull) > 0.8):
                                    hulls.append(hull)
                                    # 阈值越小，越容易逼近
                                    quad = cv2.approxPolyDP(hull, 3, True)#maximum_area_inscribed
                                    if (len(quad) == 4):
                                        areaqued = cv2.contourArea(quad)
                                        areahull = cv2.contourArea(hull)
                                        if areaqued / areahull > 0.8 and areahull >= areaqued:
                                            quads.append(quad)
                    # print("quads",quads)
                    cv2.drawContours(frame,q,-1,(0,255,0),3)      
                    cv2.drawContours(frame,contours_new_n,-1,(255,255,0),3)  
                    print(len(contours_new_n))    
                    if len(point_array_n)>0:
                        point_array_n.sort(key=lambda c: c[0], reverse=False)
                        if len(point_array_n) >= 3:
                            if len(quads) == 1:
                                point_array_n = point_array_n[:3]
                            else:
                                point_array_n = point_array_n[-3:]

                            obj = np.array([[-50, 50, 0], [50, 50, 0], [50,-50 , 0],[-50, -50 ,0]
                            ],dtype=np.float64)  # 世界坐标
                            point_four = JudgeBeveling(point_array_n[0],point_array_n[1],point_array_n[2])
                            point_array_n.append(point_four)
                        
                        elif len(point_array_n) == 2:
                                # points = point_array_n
                            dis_points = round(math.sqrt((point_array_n[0][0]-point_array_n[1][0])**2+(point_array_n[0][1]+point_array_n[1][1])**2))
                            # if dis_points > 450:
                            #     suc = False
                            # else:
                            p1 = (point_array_n[0][0],point_array_n[0][1]+height_array_n[0]/2)
                            p2 = (point_array_n[0][0],point_array_n[0][1]-height_array_n[0]/2)
                            p3 = (point_array_n[1][0],point_array_n[1][1]+height_array_n[1]/2)
                            p4 = (point_array_n[1][0],point_array_n[1][1]-height_array_n[1]/2)
                            point_array_n = [p1,p2,p3,p4]
                            obj = np.array([[-50, 25, 0], [50, 25, 0], [50, 75 , 0],[-50, 75 ,0]
                            ],dtype=np.float64)
                        else:
                            suc = False
                            
                        if suc:
                            center_x = round((point_array_n[0][0] + point_array_n[1][0] +point_array_n[2][0]+point_array_n[3][0]) / 4)
                            center_y = round((point_array_n[0][1] + point_array_n[1][1]+ point_array_n[2][1]+point_array_n[3][1]) / 4)
                            roi = [center_x-10,center_y-10,center_x+10,center_y+10]
                            # get 3D zuobiao
                            spatials, centroid = hostSpatials.calc_spatials(framedepth, roi)
                            print(spatials)
                            cx = spatials['x']
                            cy = spatials['y']
                            cz = spatials['z']
                            position = np.array([cx,cy,cz])
                            # position = np.dot(R_camera2gimbal,position) + t_camera2gimbal
                            # position = np.dot(R_gripper2base,position) + T_gripper2base
                            print(position)
                            # print(R_gripper2base)
                            point_array_n = np.int32(point_array_n)
                            # print(point_array_n)  point_array_n [[497 587][755 273][785 557][466 303]],should add one more []
                            cv2.polylines(frame,[point_array_n],True,(0,0,255),2)
                            # cv2.polylines(frame,[ore_box],True,(0,255,255),2)
                            pnts = np.array(point_array_n,dtype=np.float64) # 像素坐标
                            success,rvec, tvec = cv2.solvePnP(obj, pnts, Camera_intrinsic["mtx"], Camera_intrinsic["dist"],flags=cv2.SOLVEPNP_ITERATIVE)
                            rvec_matrix, _  = cv2.Rodrigues(rvec)
                            # rvec_matrix = np.dot(R_gripper2base,R_camera2gimbal,rvec_matrix)
                            proj_matrix = np.hstack((rvec_matrix, rvec))
                            r = R.from_matrix(rvec_matrix)
                            Quaternion = r.as_quat()
                            qx = Quaternion[0]
                            qy = Quaternion[1]
                            qz = Quaternion[2]
                            qw = Quaternion[3]

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