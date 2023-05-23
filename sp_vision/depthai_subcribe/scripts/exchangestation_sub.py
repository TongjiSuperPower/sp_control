import time
import rospy
import cv2
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import numpy as np
import math
# from math import *
from geometry_msgs.msg import Pose
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

R_camera2gimbal = np.float32([[0.9995126574971087, 0.030507826488515775, -0.006612112069085753], [-0.030487028187199994, 0.9995299636403152, 0.0032238017158466806], [0.006707355319379401, -0.0030206469732225122, 0.9999729431722053]])
t_camera2gimbal = np.float32([[-0.0], [-0.1499257336343296], [-0.26959728856029563]])


R_gripper2base = 0
T_gripper2base = 0

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
    

def getpose(pose):
    global R_gripper2base,T_gripper2base
    Gripperq=[ pose.orientation.x ,pose.orientation.y, pose.orientation.z,pose.orientation.w]
    Grippert=[[pose.position.x],[pose.position.y],[pose.position.z]]
    # print(Gripperq)
    # print(Grippert)
    Gripperq = np.array(Gripperq)
    Grippert = np.array(Grippert)
    Rm = R.from_quat(Gripperq)
    # R_gripper2base = Rm.as_matrix()
    # print("successfully")
    R_gripper2base = Rm.as_matrix()
    # print(R_gripper2base)
    # T_gripper2base = -np.dot(R_gripper2base.T,Grippert)
    # R_gripper2base = np.linalg.inv(R_gripper2base)

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
        global sub
        sub = rospy.Subscriber("calibrate",Pose,getpose,queue_size=10)
        # blue, _, red = cv2.split(frame)
        # subtracted = cv2.subtract(red, blue)
        subtracted = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        # subtracted = cv2.subtract(blue, red)
        _, threshed = cv2.threshold(subtracted, 120, 255, cv2.THRESH_BINARY)
        # _, threshed = cv2.threshold(subtracted, 120, 255, cv2.THRESH_BINARY)
        kernal = cv2.getStructuringElement(cv2.MORPH_RECT, (5, 5))
        cv2.erode(threshed, kernal, dst=threshed)
        cv2.dilate(threshed, kernal, dst=threshed)
        # cv2.imshow("hsv_image",threshed)
        contours, _ = cv2.findContours(threshed, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        contours_new = []
        point_array = []
        for contour in contours:
            if 2000 < cv2.contourArea(contour) < 8000 :
                rect = cv2.minAreaRect(contour)
                center = rect[0]
                h, w = rect[1]
                if h < w:
                    h, w = w, h
                ratio = h / w
                if 0.25 < ratio < 4:
                    contours_new.append(contour)

        quads = [] #array of quad including four peak points{}
        hulls = []
        for i in range(len(contours_new)):
            rect = cv2.minAreaRect(contours_new[i])
            center = rect[0]
            hull = cv2.convexHull(contours_new[i])
            hulls.append(hull)
            # 阈值越小，越容易逼近
            quad = cv2.approxPolyDP(hull, 3, True)# maximum_area_inscribed
            quads.append(quad)
            point_array.append(center)
        
        global qw,qx,qy,qz,cx,cy,cz
        # obj = np.array([[-112.5, 112.5, 0], [112.5, 112.5, 0], [-112.5, -112.5 , 0],[112.5, -112.5 ,0]
        #                 ],dtype=np.float64)
        cv2.drawContours(frame,quads,-1,(0, 255, 0),thickness = 2)
        
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
            rvec_matrix = np.dot(R_gripper2base,R_camera2gimbal,rvec_matrix)
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
            spatials, centroid = hostSpatials.calc_spatials(self.cv_depth, roi)
            cx = spatials['x']*0.001
            cy = spatials['y']*0.001
            cz = spatials['z']*0.001
            position = np.array([[cx],[-cy],[cz]])
            # print(position)
            position = np.dot(R_camera2gimbal,position)
            # print(position)
            position = position + t_camera2gimbal
            position = np.dot(R_gripper2base,position) + T_gripper2base
            # position = np.dot(R_gripper2base,(position - T_gripper2base)) 
            print(position)
            cx = float(position[0])
            cy = float(position[1])
            cz = float(position[2])
        # else:
        #         cx = 0
        #         cy = 0
        #         cz = 0
        #         qw = 0
        #         qx = 0
        #         qy = 0
        #         qz = 0

            objPose = Pose()
            objPose.position.x = cx
            objPose.position.y = cy
            objPose.position.z = cz
            objPose.orientation.w = qw 
            objPose.orientation.x = qx 
            objPose.orientation.y = qy 
            objPose.orientation.z = qz 
            self.target_pub.publish(objPose)
            # print(objPose)

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

        

