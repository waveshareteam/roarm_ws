import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import TransformStamped, Twist, PoseStamped
from tf2_ros import TransformBroadcaster
from scipy.spatial.transform import Rotation as Rscipy

import cv2
import numpy as np
import json
from dt_apriltags import Detector
import os
import subprocess
import signal
import time
import math

curpath = os.path.realpath(__file__)
thisPath = os.path.dirname(curpath)

try:
    existing_mediamtx_pids = subprocess.check_output(
        ["pgrep", "-f", "mediamtx"], encoding="utf-8"
    ).splitlines()

    existing_gst_launch_pids = subprocess.check_output(
        ["pgrep", "-f", "gst-launch-1.0"], encoding="utf-8"
    ).splitlines()

    for pid_str in existing_mediamtx_pids:
        pid = int(pid_str)
        print(f"Killing existing mediamtx process: {pid}")
        os.kill(pid, signal.SIGTERM) 

    for pid_str in existing_gst_launch_pids:
        pid = int(pid_str)
        print(f"Killing existing gst-launch-1.0 process: {pid}")
        os.kill(pid, signal.SIGTERM) 
except subprocess.CalledProcessError:
    pass
    
log_file_path = os.path.join(thisPath,"Mediamtx", "mediamtx.log")

with open(log_file_path, "w") as log_file:
    mediamtx_command = [
        os.path.join(thisPath,"Mediamtx", "mediamtx"),
        os.path.join(thisPath,"Mediamtx", "mediamtx.yml"),
    ]
    mediamtx_process = subprocess.Popen(
        mediamtx_command,
        stdout=log_file,
        stderr=log_file
    )
        
gst_command = [
    'gst-launch-1.0',
    'fdsrc', '!',
    'rawvideoparse', 'format=bgr', 'width=640', 'height=480', 'framerate=10/1', '!',
    'videoconvert', '!',
    'x264enc', 'bitrate=1000', 'speed-preset=ultrafast', 'tune=zerolatency', '!',
    'h264parse', '!',
    'rtspclientsink', 'location=rtsp://localhost:8554/cam', 'latency=0'
]

gst_process = subprocess.Popen(gst_command, stdin=subprocess.PIPE)

def compute_3d_translation(delta_x, delta_y, Z, K):
    fx, fy = K[0, 0], K[1, 1] 
    cx, cy = K[0, 2], K[1, 2]  
    
    delta_X = (delta_x * Z) / fx  
    delta_Y = (delta_y * Z) / fy
    
    return delta_X, delta_Y

K = np.array([
    [289.11451,   0.     , 347.23664],
    [  0.     , 289.75319, 235.67429],
    [  0.     ,   0.     ,   1.     ]
], dtype=np.float64)

class ApriltagTrackPid(Node):
    def __init__(self):
        super().__init__('apriltag_track_pid')
        # Create a subscription to the image_raw topic
        # self.image_rect_subscription = self.create_subscription(Image,'/image_rect', self.image_callback,10)
        self.image_raw_subscription = self.create_subscription(Image,'/image_raw', self.image_callback,10)
        # Create a publisher to the apriltag_track_pid/result topic
        self.apriltag_track_pid_publisher = self.create_publisher(Image, '/apriltag_track_pid/result', 10)
        # Create a CvBridge object to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()
        # Create an apriltag detector object
        self.detector = Detector(
            families='tag36h11',
            nthreads=2,
            # quad_decimate=1.0,
            # quad_sigma=0.0,
            # refine_edges=1,
            # decode_sharpening=0.25,
            # debug=0
        )

        self.tag_size = 0.024

        self.obj_pts = np.array([
            [-self.tag_size/2, -self.tag_size/2, 0],
            [ self.tag_size/2, -self.tag_size/2, 0],
            [ self.tag_size/2,  self.tag_size/2, 0],
            [-self.tag_size/2,  self.tag_size/2, 0]
        ], dtype=np.float32)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.moving = False           
        self.move_end_time = None     
        self.current_twist = Twist()
        self.create_timer(0.02, self.timer_callback)

        self.declare_parameter('cam_frame', 'camera_link')
        self.declare_parameter('tag_frame', 'object_1')
        self.cam_frame = self.get_parameter('cam_frame').value
        self.tag_frame = self.get_parameter('tag_frame').value

    def timer_callback(self):
        if self.moving and self.move_end_time is not None:
            now = self.get_clock().now().nanoseconds
            if now >= self.move_end_time:
                self.cmd_pub.publish(Twist())  
                self.moving = False
                self.move_end_time = None

    def compute_move_duration(self, y_m, min_duration=0.05, max_duration=0.3, distance_threshold=0.3):
        distance = abs(y_m)
        duration = min_duration + (max_duration - min_duration) * min(distance / distance_threshold, 1.0)
        return duration

    def image_callback(self, msg):

        # Convert the ROS Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        self.frame_count = getattr(self, "frame_count", 0)
        self.frame_count += 1
        if self.frame_count % 3 != 0:  # 
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # gray = cv2.equalizeHist(gray)
        clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8,8))
        gray = clahe.apply(gray)
        gray = cv2.GaussianBlur(gray, (3, 3), 0)

        roi = gray[140:340,120:520]

        # Detect apriltags in the image
        fx = K[0, 0]
        fy = K[1, 1]
        cx = K[0, 2]
        cy = K[1, 2]
        camera_params = (fx, fy, cx, cy)

        results = self.detector.detect(roi, False, camera_params, self.tag_size)

        # Loop through the detected apriltags
        if results:
            for r in results:
                # Get the corners of the apriltag
                corners = r.corners.astype(int)
                corners[:,0] += 120
                corners[:,1] += 140
                center_x, center_y = int(r.center[0]+120), int(r.center[1]+140)

                # --- Improved PnP with RANSAC + refine ---
                image_pts = np.array(corners, dtype=np.float32).reshape(-1, 2)
                object_pts = self.obj_pts.reshape(-1, 3).astype(np.float32)
                # dist = np.zeros((5,), dtype=np.float32)
                dist = np.array([-0.208848, 0.028006, -0.000705, -0.000820, 0.0], dtype=np.float64)
                cam = K

                success, rvec, tvec, inliers = cv2.solvePnPRansac(
                    object_pts, image_pts, cam, dist,
                    flags=cv2.SOLVEPNP_IPPE_SQUARE,
                    reprojectionError=3.0,    
                    confidence=0.99,
                    iterationsCount=200,
                )

                if not success:
                    self.get_logger().warn("solvePnPRansac failed.")
                    return
                else:
                    # 输出内点数量

                    n_inliers = 0 if inliers is None else len(inliers)
                    # self.get_logger().info(f"PnPRansac success with {n_inliers} inliers")

                    # 2. （可选）使用 LM 优化内点集进一步 refine
                    try:
                        rvec, tvec = cv2.solvePnPRefineLM(
                            object_pts[inliers[:, 0]], image_pts[inliers[:, 0]],
                            cam, dist, rvec, tvec
                        )
                    except Exception:
                        pass
                    
                    # 3. 强制旋转矩阵正交化（避免数值漂移）
                    R, _ = cv2.Rodrigues(rvec)
                    U, _, Vt = np.linalg.svd(R)
                    R_ortho = U @ Vt
                    rvec, _ = cv2.Rodrigues(R_ortho)

                    # 4. 计算重投影误差
                    proj, _ = cv2.projectPoints(object_pts, rvec, tvec, cam, dist)
                    err = np.linalg.norm(proj.reshape(-1, 2) - image_pts, axis=1)
                    # self.get_logger().info(f"mean reproj err = {err.mean():.2f}px")

                    # 5. 提取平移结果
                    x_m, y_m, z_m = tvec.flatten()
                    # self.get_logger().info(f"tvec = ({x_m:.3f}, {y_m:.3f}, {z_m:.3f})")
                    # 后续继续使用 R_ortho / tvec 发布 TF 或姿态即可

                    twist = Twist()
                    move_duration = 0.0

                    if y_m < -0.05:  # 目标太远
                        self.get_logger().info("The goal is too far, move forward a little")
                        twist.linear.x = 0.1
                        move_duration = self.compute_move_duration(y_m)

                    elif y_m > 0.02:  # 目标太近
                        self.get_logger().info("The target is too close, step back a little")
                        twist.linear.x = -0.1
                        move_duration = self.compute_move_duration(y_m)

                    else:
                        twist.linear.x = 0.0
                        move_duration = 0.0

                    # 如果需要移动
                    if move_duration > 0:
                        self.moving = True
                        self.move_end_time = self.get_clock().now().nanoseconds + int(move_duration * 1e9)
                        self.current_twist = twist
                    else:
                        # 停止
                        self.moving = False
                    # self.cmd_pub.publish(twist)
                        
                # 调整姿态方向
                R_flip = np.array([
                    [1,  0,  0],
                    [0, -1,  0],
                    [0,  0, -1]
                ])
                R = R_ortho @ R_flip

                sy = math.sqrt(R[0,0]**2 + R[1,0]**2)
                singular = sy < 1e-6

                if not singular:
                    roll  = math.atan2(R[2,1], R[2,2])
                    pitch = math.atan2(-R[2,0], sy)
                    yaw   = math.atan2(R[1,0], R[0,0])
                else:
                    roll  = math.atan2(-R[1,2], R[1,1])
                    pitch = math.atan2(-R[2,0], sy)
                    yaw   = 0
                    
                roll = 0.0  
                pitch = 0.0
                # yaw = 0.0

                # 用修正后的欧拉角重新生成旋转矩阵
                R_fixed = Rscipy.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
                R = R_fixed
                # 转换为四元数
                rot = Rscipy.from_matrix(R)
                qx, qy, qz, qw = rot.as_quat()

                tag_info = (f"ID: {r.tag_id}, Pos: ({x_m:.3f}, {y_m:.3f}, {z_m:.3f}), ")
                print(tag_info)
                # Draw a polygon around the apriltag
                cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(frame, tag_info, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                delta_X, delta_Y = compute_3d_translation(30, 4, z_m, K)

                transform = TransformStamped()
                transform.header.stamp = self.get_clock().now().to_msg()
                transform.header.frame_id = self.cam_frame    
                transform.child_frame_id = self.tag_frame  

                if x_m<0:
                    delta_X = delta_X-0.01
                if x_m>0:
                    delta_X = delta_X+0.01
                transform.transform.translation.x = float(x_m+delta_X)
                transform.transform.translation.y = float(y_m+delta_Y) 
                transform.transform.translation.z = float(z_m)                

                transform.transform.rotation.x = float(qx)
                transform.transform.rotation.y = float(qy)
                transform.transform.rotation.z = float(qz)
                transform.transform.rotation.w = float(qw)

                self.tf_broadcaster.sendTransform(transform)

        gst_process.stdin.write(frame.tobytes())
        gst_process.stdin.flush()
        # Convert the OpenCV image back to a ROS Image message
        result_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")                                                                                      
        # Publish the result image message
        self.apriltag_track_pid_publisher.publish(result_img_msg)

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    apriltag_track_pid = ApriltagTrackPid()
    # Spin the node
    rclpy.spin(apriltag_track_pid)
    # Destroy the node
    apriltag_track_pid.destroy_node()
    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == '__main__':
    # Run the main function
    main()
