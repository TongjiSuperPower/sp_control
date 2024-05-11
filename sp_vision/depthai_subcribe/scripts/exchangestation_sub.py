import time
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import math
# from math import *
from geometry_msgs.msg import Pose
from std_msgs.msg import Float64MultiArray
# from modules.calc import HostSpatialsCalc_ROS
# from calc import HostSpatialsCalc_ROS
import depthai as dai
from scipy.spatial.transform import Rotation as R


class HostSpatialsCalc_ROS:
    # We need device object to get calibration data
    def __init__(self):
        # calibData = device.readCalibration()
        # Required information for calculating spatial coordinates on the host
        # self.monoHFOV = np.deg2rad(calibData.getFov(dai.CameraBoardSocket.LEFT))
        self.monoHFOV = 1.2541936111357697

        # Values
        self.DELTA = 5
        self.THRESH_LOW = 200 # 20cm
        self.THRESH_HIGH = 30000 # 30m

    def setLowerThreshold(self, threshold_low):
        self.THRESH_LOW = threshold_low
    def setUpperThreshold(self, threshold_low):
        self.THRESH_HIGH = threshold_low
    def setDeltaRoi(self, delta):
        self.DELTA = delta

    def _check_input(self, roi, frame): # Check if input is ROI or point. If point, convert to ROI
        if len(roi) == 4: return roi
        if len(roi) != 2: raise ValueError("You have to pass either ROI (4 values) or point (2 values)!")
        # Limit the point so ROI won't be outside the frame
        self.DELTA = 5 # Take 10x10 depth pixels around point for depth averaging
        x = min(max(roi[0], self.DELTA), frame.shape[1] - self.DELTA)
        y = min(max(roi[1], self.DELTA), frame.shape[0] - self.DELTA)
        return (x-self.DELTA,y-self.DELTA,x+self.DELTA,y+self.DELTA)

    def _calc_angle(self, frame, offset):
        return math.atan(math.tan(self.monoHFOV / 2.0) * offset / (frame.shape[1] / 2.0))

    # roi has to be list of ints
    def calc_spatials(self, depthFrame, roi, averaging_method=np.mean):
        roi = self._check_input(roi, depthFrame) # If point was passed, convert it to ROI
        xmin, ymin, xmax, ymax = roi

        # Calculate the average depth in the ROI.
        depthROI = depthFrame[ymin:ymax, xmin:xmax]
        inRange = (self.THRESH_LOW <= depthROI) & (depthROI <= self.THRESH_HIGH)

        averageDepth = averaging_method(depthROI[inRange])

        centroid = { # Get centroid of the ROI
            'x': int((xmax + xmin) / 2),
            'y': int((ymax + ymin) / 2)
        }

        midW = int(depthFrame.shape[1] / 2) # middle of the depth img width
        midH = int(depthFrame.shape[0] / 2) # middle of the depth img height
        bb_x_pos = centroid['x'] - midW
        bb_y_pos = centroid['y'] - midH

        angle_x = self._calc_angle(depthFrame, bb_x_pos)
        angle_y = self._calc_angle(depthFrame, bb_y_pos)

        spatials = {
            'z': averageDepth,
            'x': averageDepth * math.tan(angle_x),
            'y': -averageDepth * math.tan(angle_y)
        }
        return spatials, centroid


class Contour:
    def __init__(self,centerpoint,obj) -> None:
        self.centerpoint = centerpoint
        self.obj = obj
        
Camera_intrinsic = {

    "mtx": np.array([[1025.35323576971, 0, 678.7266096913569],
 [0, 1027.080747783651, 396.9785232682882],
 [0, 0, 1]],   dtype=np.double),
    "dist": np.array([[0.220473935504143, -1.294976334542626, 0.003407354582097702, -0.001096689035107743, 2.91864887650898]], dtype=np.double),

}

# R_camera2gimbal = np.float32([[1, 0, -0], [-0, 1, 0], [0, -0, 1]])
# # R_camera2gimbal = np.float32([[0.9995126574971087, 0.030507826488515775, -0.006612112069085753], [-0.030487028187199994, 0.9995299636403152, 0.0032238017158466806], [0.006707355319379401, -0.0030206469732225122, 0.9999729431722053]])
# t_camera2gimbal = np.float32([[-0.0], [-0.166], [-0.166]])
# # t_camera2gimbal = np.float32([[-0.06471684338670468], [-0.1499257336343296], [-0.26959728856029563]])

R_gripper2base = 0
T_gripper2base = 0
    

def getpose(pose):
    global R_gripper2base,T_gripper2base
    Gripperq=[ pose.orientation.x ,pose.orientation.y, pose.orientation.z,pose.orientation.w]
    Grippert=[[pose.position.x],[pose.position.y],[pose.position.z]]
    Gripperq = np.array(Gripperq)
    Grippert = np.array(Grippert)
    Rm = R.from_quat(Gripperq)
    # R_gripper2base = Rm.as_matrix()
    # print("successfully")
    R_gripper2base = Rm.as_matrix()
    # print(R_gripper2base)
    T_gripper2base = Grippert
    sub.unregister() 


class ImageConverter:
    def __init__(self):
        # 创建图像缓存相关的变量
        self.cv_image = None
        self.cv_depth = None
        self.get_image = False
        self.get_depth = False
        

        # 创建cv_bridge
        self.bridge = CvBridge()

        # 声明图像的发布者和订阅者
        self.image_pub = rospy.Publisher("exchangestation_detect_image",
                                         Image,
                                         queue_size=1)
                                         # 发布一个话题，话题类型为Image，就是检测到的目标图像
        self.target_pub = rospy.Publisher("exchangestation_detect_pose",
                                          Pose,
                                          queue_size=1)
                                          # 发布一个话题，话题类型为Pose,就是目标物体的位姿
        self.corner_pub = rospy.Publisher("corner_coordinates",
                                          Float64MultiArray,
                                          queue_size=1)
                                          # 发布角点坐标
                                                                            
        self.image_sub = rospy.Subscriber("/stereo_inertial_publisher/color/image",
                                          Image,
                                          self.callback,
                                          queue_size=1)
                                          # 订阅camera来的图像并执行callback函数
        self.depth_sub = rospy.Subscriber("/stereo_inertial_publisher/stereo/depth",
                                          Image,
                                          self.callback_depth,
                                          queue_size=1)
                                          # 订阅camera来的图像并执行callback函数

    def callback(self, data):
        # 判断当前图像是否处理完
        if not self.get_image:
            # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
            try:
                self.cv_image = self.bridge.imgmsg_to_cv2(data, "bgr8")
                # print(self.cv_image)
                # print('successfully')
            except CvBridgeError as e:
                print(e)
            
            # 设置标志，表示收到图像
            self.get_image = True
    
    def callback_depth(self, data):
        # 判断当前图像是否处理完
        if not self.get_image:
            # 使用cv_bridge将ROS的图像数据转换成OpenCV的图像格式
            try:
                self.cv_depth = self.bridge.imgmsg_to_cv2(data, "16UC1")
                # print(self.cv_image)
                # print('successfully')
            except CvBridgeError as e:
                print(e)
            
            # 设置标志，表示收到图像
            self.get_depth = True
            
    def detect_exchangestation(self):
        frame = self.cv_image
        subtracted = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # COLOR_BGR2GRAY
        # blue 好,gray->二值化
        _, threshed = cv2.threshold(subtracted, 50, 255, cv2.THRESH_BINARY)
        # red 暗一点, 不确定
        # _, threshed = cv2.threshold(frame[:,:,2], 50, 255, cv2.THRESH_BINARY)
        # (50,255),二值化分割
        #腐蚀
        kernal1 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
        cv2.erode(threshed, kernal1, dst=threshed)
        #膨胀
        kernal2 = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))
        cv2.dilate(threshed, kernal2, dst=threshed)
        #显示膨胀后二值化的图像
        #!!!!!!!!!!!!!!!!!!!!!
        cv2.imshow('ba',threshed)
        # 找轮廓
        contours, _ = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        # cv2.drawContours(frame,contours,-1,(0, 255, 255),thickness = 2)
        contours_new = []
        point_array = []
        quads = [] #array of quad including four peak points{}
        distance = []
        hulls = [] # hulls:所有合法的凸包中的点的集合
        
        stmp = 100000 # 最小的区域
        sarea,tid=[],0 # 最小区域的点集和编号
        for id,contour in enumerate(contours):
            if  300 < cv2.contourArea(contour) < 13000 :
                rect = cv2.minAreaRect(contour)
                h, w = rect[1]
                center = rect[0]
                if h < w:
                    h, w = w, h
                ratio = h / w
                if ratio < 5 :
                    hull = cv2.convexHull(contour)
                    # 阈值越小，越容易逼近
                    quad = cv2.approxPolyDP(hull, 7, True)# maximum_area_inscribed
                    if len(quad) >= 4:
                        spatials, centroid = hostSpatials.calc_spatials(self.cv_depth, [int(center[0]-10),int(center[1]-10),int(center[0]+10),int(center[1]+10)])
                        if spatials['z'] < 1000:
                            if cv2.contourArea(contour) < stmp:
                                stmp = cv2.contourArea(contour)
                                sarea = []
                                for points in quad:
                                    for point in points:
                                        sarea.append(point)
                                tid = id
                            quads.append(quad)
                            for points in quad:
                                for point in points:
                                    hulls.append(point)
                    # cv2.polylines(frame, [quad], True, (0, 255, 0), 2)
        hulls = np.array(hulls)
        if len(hulls) > 2 :
            epsilon = 7  # 精度参数，值越小，近似的多边形越接近原始点集
            closed = True  # 指示结果是否应该是封闭的
            approx = cv2.convexHull(hulls, epsilon, closed)
            app = []
            for tmp in approx:
                app.append(tmp[0])
            for tt in range(10): #调整凸包
                i=0
                while i < len(app):
                    pre = (i+len(app)-1)%len(app)
                    nxt = (i+1)%len(app)
                    p1 = app[i] - app[pre]
                    p2 = app[i] - app[nxt]
                    theta = math.acos(max(-1,min(1,np.dot(p1,p2)/(np.linalg.norm(p1)*np.linalg.norm(p2)))))
                    len1 = np.linalg.norm(p1)
                    len2 = np.linalg.norm(p2)
                    # !!!!!!!!!!!!!!!弹点，如果是近似一条直线 0----0----0  ->  0------------0
                    # 140可以调整
                    if theta > math.pi/180*140: # or (theta > math.pi/2 and (len1 < 100 or len2 < 100))
                        del app[i]
                        i-=1
                    i+=1
            
            aver_point = np.mean(sarea, axis=0)

            app = np.array(app)
            ruid = 0  # right up id 右上角点
            # 右上角点是蓝色
            # 其他三个点是红色
            for id, point in enumerate(app):
                if np.linalg.norm(aver_point-app[ruid]) > np.linalg.norm(aver_point-app[id]):
                    ruid = id
            for id, point in enumerate(app):
                if ruid != id:
                    cv2.circle(frame,point, 10, (0, 0, 255), -1)
            cv2.circle(frame,app[ruid], 10, (255, 0, 0), -1)
            # cv2.circle(frame,(100,10), 10, (255, 0, 0), -1)

            cv2.polylines(frame, [app], True, (0, 255, 0), 2)
            cv2.imshow("a",frame)
            if len(app) ==4:#矩形四个点才会solve pnp
                # print('######################')
                point_array=app
                point_array = np.roll(point_array,4-ruid,axis=0)
                # print(point_array)
                point_rt = Contour(point_array[0],[137.5, -137.5, 0])
                point_lt = Contour(point_array[1],[-137.5, -137.5, 0])
                point_lb = Contour(point_array[2],[-137.5, 137.5, 0])
                point_rb = Contour(point_array[3],[137.5, 137.5, 0])
                obj = np.array([point_rt.obj,point_lt.obj,point_lb.obj,point_rb.obj
                            ],dtype=np.float64)
                cornor = np.array([point_rt.obj[0],point_rt.obj[1],
                                   point_lt.obj[0],point_lt.obj[1],
                                   point_lb.obj[0],point_lb.obj[1],
                                   point_rb.obj[0],point_rb.obj[1]])
                
                point_array_n = point_array
                point_array_n = np.int32(point_array_n)
                pnts = np.array(point_array_n,dtype=np.float64) # 像素坐标
                success, rvec, tvec = cv2.solvePnP(obj, pnts, Camera_intrinsic["mtx"], Camera_intrinsic["dist"],flags=cv2.SOLVEPNP_ITERATIVE)
                
                rvec_matrix = cv2.Rodrigues(rvec)[0]
                r = R.from_matrix(rvec_matrix)
                Quaternion = r.as_quat()
                Euler = r.as_euler('xyz', degrees=True)
                # Euler = r.as_euler('xyz', degrees=True)
                print(Euler)
                objPose = Pose()
                objPose.position.x = tvec[0]
                objPose.position.y = tvec[1]
                objPose.position.z = tvec[2]
                objPose.orientation.w = Quaternion[0] 
                objPose.orientation.x = Quaternion[1]
                objPose.orientation.y = Quaternion[2]
                objPose.orientation.z = Quaternion[3]
                self.target_pub.publish(objPose)
                self.corner_pub.publish(cornor)
                print(objPose)
              
       
        
            # # get 3D zuobiao
            # spatials, centroid = hostSpatials.calc_spatials(self.cv_depth, roi)
            # cx = spatials['x']*0.001
            # cy = spatials['y']*0.001
            # cz = spatials['z']*0.001
            # print(spatials)
            # position = np.array([[cx],[-cy],[cz]])
            # position = np.dot(R_camera2gimbal,position)
            # position = position + t_camera2gimbal
            # position = np.dot(R_gripper2base,position) + T_gripper2base
            # position = np.dot(R_gripper2base,(position - T_gripper2base)) 
            # print(position)
            # cx = float(position[0])
            # cy = float(position[1])
            # cz = float(position[2])

            # objPose = Pose()
            # objPose.position.x = cx
            # objPose.position.y = cy
            # objPose.position.z = cz
            # objPose.orientation.w = -math.sqrt(2)/2 
            # objPose.orientation.x = math.sqrt(2)/2 
            # objPose.orientation.y = 0
            # objPose.orientation.z = 0

        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)
            
    def loop(self):
        if self.get_image:
            self.detect_exchangestation()
            self.get_image = False

        
if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("exchangestation_detect")
        rospy.loginfo("Starting detect exchangestation")
        image_converter = ImageConverter()
        rate = rospy.Rate(100)
        hostSpatials = HostSpatialsCalc_ROS()
        delta = 5
        hostSpatials.setDeltaRoi(delta)
        while not rospy.is_shutdown():
            image_converter.loop()
            rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down exchangeststion_detect node.")
        cv2.destroyAllWindows()

        

