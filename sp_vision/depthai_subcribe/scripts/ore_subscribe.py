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

params = {'hue_low': 8, 'saturation_low': 110, 'value_low': 0,
          'hue_high': 30, 'saturation_high': 255, 'value_high': 255,
          'min_area_size': 5000, 'max_area_size': 400000}
thresh_low = (params['hue_low'], params['saturation_low'], params['value_low'])
thresh_high = (params['hue_high'], params['saturation_high'], params['value_high'])
min_area_size = params['min_area_size']
max_area_size = params['max_area_size']
lower_black = np.array([0,0,0]) 
upper_black= np.array([180, 255, 43])
Camera_intrinsic = {

    "mtx": np.array([[1025.35323576971, 0, 678.7266096913569],
 [0, 1027.080747783651, 396.9785232682882],
 [0, 0, 1]],   dtype=np.double),
    "dist": np.array([[0.220473935504143, -1.294976334542626, 0.003407354582097702, -0.001096689035107743, 2.91864887650898]], dtype=np.double),

}

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

def find_ore(color_image):
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
        self.image_pub = rospy.Publisher("ore_detect_image",
                                         Image,
                                         queue_size=1)
                                         # 发布一个话题，话题类型为Image，就是检测到的目标图像
        self.target_pub = rospy.Publisher("ore_detect_pose",
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

    def detect_ore(self):

        frame = self.cv_image
        ore_box = find_ore(frame)
        if ore_box is not None:

            # 高斯滤波，对图像邻域内像素进行平滑
            hsv_image = cv2.GaussianBlur(self.cv_image, (5, 5), 0)

            # 颜色空间转换，将RGB图像转换成HSV图像
            hsv_image = cv2.cvtColor(hsv_image, cv2.COLOR_BGR2HSV)
            
            # # 根据阈值，去除背景
            mask = cv2.inRange(hsv_image, lower_black, upper_black)
            # output = cv2.bitwise_and(self.cv_image, self.cv_image, mask=mask)

            # # 将彩色图像转换成灰度图像
            # cvImg = cv2.cvtColor(output, 6)  # cv2.COLOR_BGR2GRAY
            # npImg = np.asarray(cvImg)
            # thresh = cv2.threshold(npImg, 1, 255, cv2.THRESH_BINARY)[1]
            # # 对识别后的图像进行腐蚀与膨胀，消除较小的连通域 交叉型 7x7 效果较好
            kernal = cv2.getStructuringElement(cv2.MORPH_CROSS, (15, 15))
            # 先膨胀后腐蚀
            cv2.dilate(mask, kernal, dst=mask)
            cv2.erode(mask, kernal, dst=mask)

            # 检测目标物体的轮廓
            # cnts, hierarchy = cv2.findContours(thresh, cv2.RETR_LIST,
            #                                         cv2.CHAIN_APPROX_NONE)
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
                # for i in range(len(contours_new)):
                #     # print(len(contours_new[i]))  
                #     # 不知道为什么在方形轮廓点数为50+ ，可能要多边形逼近            
                #     if 3 < len(contours_new[i]) <90:
                #         contours_new_n.append(contours_new[i])
                #         point_array_n.append(point_array[i]) 
                quads = [] #array of quad including four peak points
                hulls = []
                
                suc = True
                for i in range(len(contours_new_n)):
                    if (hierarchy[0, i, 3] < 0 and contours_new_n[i].shape[0] >= 4):
                        area = cv2.contourArea(contours_new_n[i])
                        # if 2000 < area < 8000 :
                        hull = cv2.convexHull(contours_new_n[i])
                        if (area / cv2.contourArea(hull) > 0.8):
                            hulls.append(hull)
                            # 阈值越小，越容易逼近
                            quad = cv2.approxPolyDP(hull, 1, True)#maximum_area_inscribed
                            if (len(quad) == 4):
                                areaqued = cv2.contourArea(quad)
                                areahull = cv2.contourArea(hull)
                                if areaqued / areahull > 0.8 and areahull >= areaqued:
                                    quads.append(quad)
                                    
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
                        if dis_points > 450:
                            suc = False
                        else:
                            p1 = (point_array_n[0][0],point_array_n[0][1]+height_array_n[0]/2)
                            p2 = (point_array_n[0][0],point_array_n[0][1]-height_array_n[0]/2)
                            p3 = (point_array_n[1][0],point_array_n[1][1]+height_array_n[1]/2)
                            p4 = (point_array_n[1][0],point_array_n[1][1]-height_array_n[1]/2)
                            point_array_n = [p1,p2,p3,p4]
                            # cen_x = round((point_array_n[0][0] +point_array_n[1][0])/2)
                            # cen_y = round((point_array_n[0][1] +point_array_n[1][1])/2)
                            # point_array_n = rota_rect(cen_x,cen_y,90-angle_array[0],200,80)
                            obj = np.array([[-50, 25, 0], [50, 25, 0], [50, 75 , 0],[-50, 75 ,0]
                            ],dtype=np.float64)
                    else:
                        suc = False
                    
                    global qw,qx,qy,qz,cx,cy,cz
                        
                    if suc:
                        center_x = round((point_array_n[0][0] + point_array_n[1][0] +point_array_n[2][0]+point_array_n[3][0]) / 4)
                        center_y = round((point_array_n[0][1] + point_array_n[1][1]+ point_array_n[2][1]+point_array_n[3][1]) / 4)
                        roi = [center_x-10,center_y-10,center_x+10,center_y+10]
                        # get 3D zuobiao
                        spatials, centroid = hostSpatials.calc_spatials(self.cv_depth, roi)
                        cx = spatials['x']
                        cy = spatials['y']
                        cz = spatials['z']
                        point_array_n = np.int32(point_array_n)
                        pnts = np.array(point_array_n,dtype=np.float64) # 像素坐标
                        success,rvec, tvec = cv2.solvePnP(obj, pnts, Camera_intrinsic["mtx"], Camera_intrinsic["dist"],flags=cv2.SOLVEPNP_ITERATIVE)
                        rvec_matrix = cv2.Rodrigues(rvec)[0]
                        proj_matrix = np.hstack((rvec_matrix, rvec))
                        eulerAngles = -cv2.decomposeProjectionMatrix(proj_matrix)[6]  # 欧拉角
                        pitch, yaw, roll = eulerAngles[0], eulerAngles[1], eulerAngles[2]
                        rot_params = [roll, pitch, yaw]  # 欧拉角 数组
                        Quaternion = EulerAndQuaternionTransform(rot_params)
                        print(0)
                        qw = Quaternion[0]
                        qx = Quaternion[1]
                        qy = Quaternion[2]
                        qz = Quaternion[3]
                    # else:
                    #     cx = 0
                    #     cy = 0
                    #     cz = 0
                    #     qw = 0
                    #     qx = 0
                    #     qy = 0
                    #     qz = 0

            # # 遍历找到的所有轮廓线
            # for c in cnts:

            #     # 去除一些面积太小的噪声
            #     if c.shape[0] < 150:
            #         continue

            #     # 提取轮廓的特征
            #     M = cv2.moments(c)

            #     if int(M["m00"]) not in range(500, 22500):
            #         continue

            #     cX = int(M["m10"] / M["m00"])
            #     cY = int(M["m01"] / M["m00"])

            #     print("x: {}, y: {}, size: {}".format(cX, cY, M["m00"]))

                # 把轮廓描绘出来，并绘制中心点
                # cv2.drawContours(self.cv_image, [c], -1, (0, 0, 255), 2)
                        cv2.drawContours(self.cv_image, contours_new_n, -1, (0, 0, 255), 2)

                        # cv2.circle(self.cv_image, (cX, cY), 1, (0, 0, 255), -1)
                        
                        # 将目标位置通过话题发布
                        objPose = Pose()
                        objPose.position.x = cx
                        objPose.position.y = cy
                        objPose.position.z = cz
                        objPose.orientation.w = qw 
                        objPose.orientation.x = qx 
                        objPose.orientation.y = qy 
                        objPose.orientation.z = qz 
                        self.target_pub.publish(objPose)
                        print(objPose)
                # else:
                #     objPose = Pose()
                #     objPose.position.x = 0
                #     objPose.position.y = 0
                #     objPose.position.z = 0
                #     objPose.orientation.w = 0 
                #     objPose.orientation.x = 0
                #     objPose.orientation.y = 0 
                #     objPose.orientation.z = 0 
                #     # objPose.position.y = cY
                    # objPose.position.z = M["m00"]
                

        # 再将opencv格式额数据转换成ros image格式的数据发布
        try:
            self.image_pub.publish(
                self.bridge.cv2_to_imgmsg(self.cv_image, "bgr8"))
        except CvBridgeError as e:
            print(e)

    def loop(self):
        if self.get_image:
            self.detect_ore()
            self.get_image = False


if __name__ == '__main__':
    try:
        # 初始化ros节点
        rospy.init_node("ore_detect")
        rospy.loginfo("Starting detect ore")
        image_converter = ImageConverter()
        rate = rospy.Rate(100)
        hostSpatials = HostSpatialsCalc_ROS()
        delta = 5
        hostSpatials.setDeltaRoi(delta)
        while not rospy.is_shutdown():
            image_converter.loop()
            rate.sleep()
    except KeyboardInterrupt:
        print("Shutting down ore_detect node.")
        cv2.destroyAllWindows()