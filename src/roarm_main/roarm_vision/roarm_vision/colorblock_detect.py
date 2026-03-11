import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String,Header
from cv_bridge import CvBridge
from geometry_msgs.msg import Point,TransformStamped
from builtin_interfaces.msg import Time
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from scipy.spatial.transform import Rotation as Rscipy
from rcl_interfaces.msg import ParameterDescriptor

import cv2
import numpy as np
import json
import os
import yaml
import subprocess
import signal
import time
import math
from math import isnan
from collections import deque

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

K = np.array([
    [289.11451,   0.     , 347.23664],
    [  0.     , 289.75319, 235.67429],
    [  0.     ,   0.     ,   1.     ]
], dtype=np.float64)

def compute_3d_translation(delta_x, delta_y, Z, K):
    fx, fy = K[0, 0], K[1, 1]  
    cx, cy = K[0, 2], K[1, 2] 
    
    delta_X = (delta_x * Z) / fx 
    delta_Y = (delta_y * Z) / fy 
    
    return delta_X, delta_Y

def order_points(pts):
    rect = np.zeros((4, 2), dtype="float32")
    s = pts.sum(axis=1)
    rect[0] = pts[np.argmin(s)]     # top-left
    rect[2] = pts[np.argmax(s)]     # bottom-right
    diff = np.diff(pts, axis=1)
    rect[1] = pts[np.argmin(diff)]  # top-right
    rect[3] = pts[np.argmax(diff)]  # bottom-left
    return rect

class ColorBlockDetect(Node):
    def __init__(self):
        super().__init__('colorblock_detect')
        # Create a subscription to the image_raw topic
        # self.image_rect_subscription = self.create_subscription(Image,'/image_rect', self.image_callback,10)
        self.image_raw_subscription = self.create_subscription(Image,'/image_raw', self.image_callback,10)
        # Create a publisher to the colorblock_detect/result topic
        self.colorblock_detect_publisher = self.create_publisher(Image, '/colorblock_detect/result', 10)
        # Create a CvBridge object to convert between ROS Image messages and OpenCV images
        self.bridge = CvBridge()


        # Declare parameters for lower and upper hue, saturation, and value
        self.declare_parameter("lower_l", 110, ParameterDescriptor(description="Lower L"))
        self.declare_parameter("lower_a", 0, ParameterDescriptor(description="Lower A"))
        self.declare_parameter("lower_b", 160, ParameterDescriptor(description="Lower B"))
        
        self.declare_parameter("upper_l", 255, ParameterDescriptor(description="Upper L"))
        self.declare_parameter("upper_a", 110, ParameterDescriptor(description="Upper A"))
        self.declare_parameter("upper_b", 255, ParameterDescriptor(description="Upper B"))
            
        # Initialize the lower and upper color arrays with the parameter values
        self.lower_color = np.array([self.get_parameter("lower_l").value, 
                                     self.get_parameter("lower_a").value, 
                                     self.get_parameter("lower_b").value])
        self.upper_color = np.array([self.get_parameter("upper_l").value, 
                                     self.get_parameter("upper_a").value, 
                                     self.get_parameter("upper_b").value])

        self.declare_parameter('cam_frame', 'camera_link')
        self.declare_parameter('tag_frame', 'object_1')
        self.cam_frame = self.get_parameter('cam_frame').value
        self.tag_frame = self.get_parameter('tag_frame').value

        self.tag_size_w = 0.026
        self.tag_size_h = 0.071

        self.obj_pts = np.array([
            [-self.tag_size_w/2, -self.tag_size_h/2, 0],
            [ self.tag_size_w/2, -self.tag_size_h/2, 0],
            [ self.tag_size_w/2,  self.tag_size_h/2, 0],
            [-self.tag_size_w/2,  self.tag_size_h/2, 0]
        ], dtype=np.float32)

        self.tf_broadcaster = TransformBroadcaster(self)

        self.add_on_set_parameters_callback(self.on_param_change)

    def on_param_change(self, params):
        for param in params:
            if param.name in (
                "lower_l", "lower_a", "lower_b",
                "upper_l", "upper_a", "upper_b"
            ):
                self.lower_color = np.array([
                    self.get_parameter("lower_l").value,
                    self.get_parameter("lower_a").value,
                    self.get_parameter("lower_b").value
                ], dtype=np.uint8)
    
                self.upper_color = np.array([
                    self.get_parameter("upper_l").value,
                    self.get_parameter("upper_a").value,
                    self.get_parameter("upper_b").value
                ], dtype=np.uint8)
    
                self.get_logger().info(
                    f"Updated LAB range: lower={self.lower_color}, upper={self.upper_color}"
                )
    
        return SetParametersResult(successful=True)
        
    def image_callback(self, msg):

        # Convert the ROS Image message to an OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

        self.frame_count = getattr(self, "frame_count", 0)
        self.frame_count += 1
        if self.frame_count % 3 != 0:  # 
            return

        img_h, img_w = frame.shape[:2]
        lab = cv2.cvtColor(frame, cv2.COLOR_BGR2LAB)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        mask = cv2.inRange(lab, self.lower_color, self.upper_color)

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            c = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(c)
            if area > 100:
                rect = cv2.minAreaRect(c)
                (x, y), (w, h), angle = rect

                aspect_ratio = max(w, h) / min(w, h)
                if aspect_ratio < 4.0:  
                    center_x, center_y = int(x), int(y)

                    corners = cv2.boxPoints(rect)
                    corners = np.int0(corners)
                    # self.get_logger().info(f'Tracking ball at ({center_x}, {center_y}), area={area:.1f}')

                    corners = order_points(corners).astype(np.float32)                        
                    corners = corners.astype(np.float32)
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)
                    print(f"Corners before cornerSubPix: {corners}")

                    # Check if corners are within image bounds
                    if np.any(corners < 0) or np.any(corners >= frame.shape[:2]):
                        print("Error: Corner points are out of bounds!")
                        return
                    
                    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.01)

                    cv2.cornerSubPix(gray, corners, (5,5), (-1,-1), criteria)

                    # --- Improved PnP with RANSAC + refine ---
                    image_pts = np.array(corners, dtype=np.float32).reshape(-1, 2)
                    object_pts = self.obj_pts.reshape(-1, 3).astype(np.float32)
                    # dist = np.zeros((5,), dtype=np.float32)
                    dist = np.array([-0.208848, 0.028006, -0.000705, -0.000820, 0.0], dtype=np.float64)
                    cam = K

                    # 1. First, use RANSAC to find a robust solution.
                    success, rvec, tvec, inliers = cv2.solvePnPRansac(
                        object_pts, image_pts, cam, dist,
                        flags=cv2.SOLVEPNP_IPPE_SQUARE,
                        reprojectionError=3.0,     # Adjustable
                        confidence=0.99,
                        iterationsCount=200,
                    )

                    if not success:
                        self.get_logger().warn("solvePnPRansac failed.")
                        return
                    else:
                        # Output the number of interior points

                        n_inliers = 0 if inliers is None else len(inliers)
                        # self.get_logger().info(f"PnPRansac success with {n_inliers} inliers")

                        # 2. (Optional) Further refine the interior point set using LM optimization.
                        try:
                            rvec, tvec = cv2.solvePnPRefineLM(
                                object_pts[inliers[:, 0]], image_pts[inliers[:, 0]],
                                cam, dist, rvec, tvec
                            )
                        except Exception:
                            pass
                        
                        # 3. Force rotation matrix orthogonalization (to avoid numerical drift)
                        R, _ = cv2.Rodrigues(rvec)
                        U, _, Vt = np.linalg.svd(R)
                        R_ortho = U @ Vt
                        rvec, _ = cv2.Rodrigues(R_ortho)

                        # 4. Calculate reprojection error
                        proj, _ = cv2.projectPoints(object_pts, rvec, tvec, cam, dist)
                        err = np.linalg.norm(proj.reshape(-1, 2) - image_pts, axis=1)
                        # self.get_logger().info(f"mean reproj err = {err.mean():.2f}px")

                        # 5. Extract translation result
                        x_m, y_m, z_m = tvec.flatten()
                        # self.get_logger().info(f"tvec = ({x_m:.3f}, {y_m:.3f}, {z_m:.3f})")

                        # Continue using R_ortho / tvec to publish TF or poses.
                            
                        # Adjust attitude and direction
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

                        # Regenerate the rotation matrix using the corrected Euler angles.
                        R_fixed = Rscipy.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
                        R = R_fixed
                        # Convert to quaternion
                        rot = Rscipy.from_matrix(R)
                        qx, qy, qz, qw = rot.as_quat()

                    #     tag_info = (f"ID: {r['id']}, Pos: ({x_m:.3f}, {y_m:.3f}, {z_m:.3f}), "
                    # f"Ori: ({roll:.3f}, {pitch:.3f}, {yaw:.3f})")
                        tag_info = (f"Pos: ({x_m:.3f}, {y_m:.3f}, {z_m:.3f})")
                        print(tag_info)
                        # Draw a polygon around the apriltag
                        cv2.polylines(frame, [corners.astype(np.int32)], isClosed=True, color=(0, 255, 0), thickness=2)
                        cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                        cv2.putText(frame, tag_info, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

                        delta_X, delta_Y = compute_3d_translation(30, 4, z_m, K)

                        transform = TransformStamped()
                        transform.header.stamp = self.get_clock().now().to_msg()
                        transform.header.frame_id = self.cam_frame     
                        # transform.child_frame_id = f"object_{r['id']}"  
                        transform.child_frame_id = self.tag_frame

                        # if x_m<0:
                        #     delta_X = delta_X-0.01
                        # if x_m>0:
                        #     delta_X = delta_X+0.01
                        transform.transform.translation.x = float(x_m+delta_X)
                        transform.transform.translation.y = float(y_m+delta_Y) 
                        # cam in hand
                        transform.transform.translation.z = float(z_m)                

                        transform.transform.rotation.x = float(qx)
                        transform.transform.rotation.y = float(qy)
                        transform.transform.rotation.z = float(qz)
                        transform.transform.rotation.w = float(qw)

                        self.tf_broadcaster.sendTransform(transform)

        gst_process.stdin.write(frame.tobytes())
        # Convert the OpenCV image back to a ROS Image message
        result_img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")                                                                                      
        # Publish the result image message
        self.colorblock_detect_publisher.publish(result_img_msg)

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    colorblock_detect = ColorBlockDetect()
    # Spin the node
    rclpy.spin(colorblock_detect)
    # Destroy the node
    colorblock_detect.destroy_node()
    # Shutdown the ROS client library
    rclpy.shutdown()

if __name__ == '__main__':
    # Run the main function
    main()

