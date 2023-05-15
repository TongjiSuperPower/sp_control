import time
from pathlib import Path
import cv2
import depthai as dai
from geometry_msgs.msg import Pose
from scipy.spatial.transform import Rotation as R
import rospy
import numpy as np

R_gripper2base = []
T_gripper2base = []
# R_target2cam = []
# t_target2cam = []
points_3d = [] # 三维坐标
points_2d = [] # 二维坐标
def calibrate_camera(img):
    '''
    通过相机标定获得cameraMatrix和distcoeffs
    通过手眼标定获得R_camera2gimbal和t_camera2gimbal
    '''
    # print("R_gripper2base",R_gripper2base)
    # print("T_gripper2base",T_gripper2base)
    w = 10  # 棋盘格角点每行数量
    h = 8  # 棋盘格角点每列数量
    a = 8 # cm or mm
    # while True:
    # 生成棋盘格三维坐标
    obj_points = np.zeros(((w*h), 3), np.float32)
    obj_points[:, :2] = np.mgrid[0:w*a:a , 0:h*a:a].T.reshape(-1, 2)  # 将纯0数组进行编码，编码代表每一个角点的位置信息，例如[0., 0., 0.],[1., 0., 0.]
    obj_points = np.reshape(obj_points, (w*h, 1, 3))  # 将位置信息矩阵变为w*h个1行三列的矩阵
    # 计算棋盘格内角点的三维坐标及其在图像中的二维坐标
    img_copy = img.copy()
    img_gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)  # 转为灰度图
    begin = time.time()
    ret, corners1 = cv2.findChessboardCorners(img_gray, (w, h))  # 寻找内角点
    if ret == True:  # 如果寻找到足够数量的内焦点
        _, corners2 = cv2.find4QuadCornerSubpix(img_gray, corners1, (5, 5))  # 细化内角点
        cv2.drawChessboardCorners(img_copy,(w,h),corners2,ret)
        
        end = time.time()
        print((end-begin)/60) 
        points_3d.append(obj_points)  # 计算三维坐标
        points_2d.append(corners2)  # 计算二维坐标
    else:
        end = time.time()
        # print((end-begin)/60)
    cv2.imshow('a',img_copy)




def getpose(pose):
    Gripperq=[pose.orientation.w, pose.orientation.x ,pose.orientation.y, pose.orientation.z]
    Grippert=[pose.position.x,pose.position.y,pose.position.z]
    Gripperq = np.array(Gripperq)
    Grippert = np.array(Grippert)
    Rm = R.from_quat(Gripperq)
    # R_gripper2base = Rm.as_matrix()
    print("successfully")
    R_gripper2base.append(Rm.as_matrix())
    # T_gripper2base = Grippert
    T_gripper2base.append(Grippert)
    sub.unregister() 




# Create pipeline
pipeline = dai.Pipeline()

camRgb = pipeline.create(dai.node.ColorCamera)
camRgb.setBoardSocket(dai.CameraBoardSocket.RGB)
camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_4_K)

xoutRgb = pipeline.create(dai.node.XLinkOut)
xoutRgb.setStreamName("rgb")
camRgb.video.link(xoutRgb.input)

xin = pipeline.create(dai.node.XLinkIn)
xin.setStreamName("control")
xin.out.link(camRgb.inputControl)

# Properties
videoEnc = pipeline.create(dai.node.VideoEncoder)
videoEnc.setDefaultProfilePreset(1, dai.VideoEncoderProperties.Profile.MJPEG)
camRgb.still.link(videoEnc.input)

# Linking
xoutStill = pipeline.create(dai.node.XLinkOut)
xoutStill.setStreamName("still")
videoEnc.bitstream.link(xoutStill.input)

rospy.init_node("cali_listener")
#3.实例化 订阅者 对象
sub = rospy.Subscriber("calibrate",Pose,getpose,queue_size=10)

# Connect to device and start pipeline
with dai.Device(pipeline) as device:

    # Output queue will be used to get the rgb frames from the output defined above
    qRgb = device.getOutputQueue(name="rgb", maxSize=30, blocking=False)
    qStill = device.getOutputQueue(name="still", maxSize=30, blocking=True)
    qControl = device.getInputQueue(name="control")

    # Make sure the destination path is present before starting to store the examples
    dirName = "rgb_data"
    Path(dirName).mkdir(parents=True, exist_ok=True)

    while True:
        inRgb = qRgb.tryGet()  # Non-blocking call, will return a new data that has arrived or None otherwise
        if inRgb is not None:
            frame = inRgb.getCvFrame()
            # 4k / 4
            frame = cv2.pyrDown(frame)
            frame = cv2.pyrDown(frame)
            cv2.imshow("rgb", frame)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        if key == ord('c'):
            ctrl = dai.CameraControl()
            ctrl.setCaptureStill(True)
            qControl.send(ctrl)
            print("Sent 'still' event to the camera!")
            sub = rospy.Subscriber("calibrate",Pose,getpose,queue_size=10) 
            calibrate_camera(frame)

        if qStill.has():
            fName = f"{dirName}/{int(time.time() * 1000)}.jpeg"
            with open(fName, "wb") as f:
                f.write(qStill.get().getData())
                print('Image saved to', fName)
            
        elif key == ord('a'):
            # 相机标定
            h, w, _ = frame.shape #  获取图像尺寸
            _, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(points_3d, points_2d, (w, h), None, None)
            print(f'cameraMatrix = np.float32({mtx.tolist()})')
            print(f'distCoeffs = np.float32({dist[0].tolist()})')

            # 手眼标定
            R_target2cam = rvecs
            t_target2cam = tvecs
            print(R_gripper2base)
            print(T_gripper2base)
            print(R_target2cam)
            print(t_target2cam)
            R_gripper2base = R_gripper2base[1:]
            T_gripper2base = T_gripper2base[1:]
            R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(R_gripper2base, T_gripper2base, R_target2cam, t_target2cam)

            print(f'R_camera2gimbal = np.float32({R_cam2gripper.tolist()})')
            print(f't_camera2gimbal = np.float32({t_cam2gripper.tolist()})')

            # 重投影误差
            mean_error = 0
            for i in range(len(points_3d)):
                projected_points, _ = cv2.projectPoints(
                    points_3d[i],
                    rvecs[i],
                    tvecs[i],
                    mtx,
                    dist
                )
                error = cv2.norm(points_2d[i], projected_points, cv2.NORM_L2) / len(projected_points)
                mean_error += error
            mean_error /= len(points_3d)
            print(f"# 重投影误差: {mean_error:.4f}px")

        