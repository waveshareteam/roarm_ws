#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from depthai_ros_msgs.msg import SpatialDetectionArray, SpatialDetection
from cv_bridge import CvBridge
import cv2
import depthai as dai
import numpy as np
import time
from std_msgs.msg import Header
from builtin_interfaces.msg import Time
from vision_msgs.msg import ObjectHypothesis, BoundingBox2D, Pose2D, Point2D
from geometry_msgs.msg import Twist,Point, TransformStamped
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from scipy.spatial.transform import Rotation as Rscipy

import json
import os
import yaml
import subprocess
import signal
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
    'rawvideoparse', 'format=bgr', 'width=640', 'height=640', 'framerate=10/1', '!',
    'videoconvert', '!',
    'x264enc', 'bitrate=1000', 'speed-preset=ultrafast', 'tune=zerolatency', '!',
    'h264parse', '!',
    'rtspclientsink', 'location=rtsp://localhost:8554/cam', 'latency=0'
]

gst_process = subprocess.Popen(gst_command, stdin=subprocess.PIPE)

class OakYoloNode(Node):
    def __init__(self):
        super().__init__('oak_yolo_node')
        self.bridge = CvBridge()
        self.detection_pub = self.create_publisher(SpatialDetectionArray, '/detections', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.create_pipeline()

        self.timer = self.create_timer(0.03, self.process_frames)  # 30 FPS

    def create_pipeline(self):
        pipeline = dai.Pipeline()
        camRgb = pipeline.create(dai.node.ColorCamera)
        spatialDetectionNetwork = pipeline.create(dai.node.YoloSpatialDetectionNetwork)
        monoLeft = pipeline.create(dai.node.MonoCamera)
        monoRight = pipeline.create(dai.node.MonoCamera)
        stereo = pipeline.create(dai.node.StereoDepth)

        xoutRgb = pipeline.create(dai.node.XLinkOut)
        xoutNN = pipeline.create(dai.node.XLinkOut)
        xoutDepth = pipeline.create(dai.node.XLinkOut)

        xoutRgb.setStreamName("rgb")
        xoutNN.setStreamName("detections")
        xoutDepth.setStreamName("depth")

        camRgb.setPreviewSize(640, 640)
        camRgb.setPreviewKeepAspectRatio(False)
        camRgb.setResolution(dai.ColorCameraProperties.SensorResolution.THE_1080_P)
        camRgb.setInterleaved(False)
        camRgb.setColorOrder(dai.ColorCameraProperties.ColorOrder.BGR)
        camRgb.setFps(5)

        monoLeft.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoLeft.setCamera("left")
        monoRight.setResolution(dai.MonoCameraProperties.SensorResolution.THE_400_P)
        monoRight.setCamera("right")

        stereo.setDefaultProfilePreset(dai.node.StereoDepth.PresetMode.HIGH_DENSITY)
        stereo.setLeftRightCheck(True)
        stereo.setExtendedDisparity(True)
        #stereo.initialConfig.setDisparityShift(100)
        stereo.setDepthAlign(dai.CameraBoardSocket.CAM_A)
        stereo.setOutputSize(monoLeft.getResolutionWidth(), monoLeft.getResolutionHeight())

        spatialDetectionNetwork.setBlobPath("/home/ws/roarm_ws/src/roarm_main/roarm_vision/config/model_yolov8n.blob")
        spatialDetectionNetwork.setConfidenceThreshold(0.5)
        spatialDetectionNetwork.setBoundingBoxScaleFactor(0.5)
        spatialDetectionNetwork.setDepthLowerThreshold(100)
        spatialDetectionNetwork.setDepthUpperThreshold(5000)

        spatialDetectionNetwork.setNumClasses(2)
        spatialDetectionNetwork.setCoordinateSize(4)
        spatialDetectionNetwork.setIouThreshold(0.5)
        spatialDetectionNetwork.passthrough.link(xoutRgb.input)

        monoLeft.out.link(stereo.left)
        monoRight.out.link(stereo.right)
        camRgb.preview.link(spatialDetectionNetwork.input)

        spatialDetectionNetwork.out.link(xoutNN.input)
        stereo.depth.link(spatialDetectionNetwork.inputDepth)
        spatialDetectionNetwork.passthroughDepth.link(xoutDepth.input)

        self.device = dai.Device(pipeline,usb2Mode=True)
        self.previewQueue = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
        self.detectionNNQueue = self.device.getOutputQueue(name="detections", maxSize=4, blocking=False)
        self.depthQueue = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

        # return pipeline

    def process_frames(self):
        inPreview = self.previewQueue.get()
        inDet = self.detectionNNQueue.get()
        depth = self.depthQueue.get()

        if inPreview is None or inDet is None or depth is None:
            return 

        frame = inPreview.getCvFrame()  
        height, width = frame.shape[:2]
        depthFrame = depth.getFrame()
        detections = inDet.detections
        msg = SpatialDetectionArray()
        for detection in detections:
            if detection.confidence > 0.75 and detection.spatialCoordinates.z<300:

                det_msg = SpatialDetection()
                obj_hypothesis = ObjectHypothesis()
                obj_hypothesis.class_id = str(detection.label)
                obj_hypothesis.score = detection.confidence
                det_msg.results.append(obj_hypothesis)

                bbox = BoundingBox2D()
                bbox.center = Pose2D()
                bbox.center.position = Point2D()
                bbox.center.position.x = (detection.xmin + detection.xmax) / 2
                bbox.center.position.y = (detection.ymin + detection.ymax) / 2
                bbox.size_x = detection.xmax - detection.xmin
                bbox.size_y = detection.ymax - detection.ymin
                det_msg.bbox = bbox

                det_msg.position = Point()
                det_msg.position.x = detection.spatialCoordinates.x / 1000
                det_msg.position.y = -detection.spatialCoordinates.y / 1000
                det_msg.position.z = detection.spatialCoordinates.z / 1000

                tag_info = (f"ID: {detection.label}, Pos: ({det_msg.position.x:.3f}, {det_msg.position.y:.3f}, {det_msg.position.z:.3f}), ")
                print(tag_info)
                # Draw a polygon around the apriltag
                x1 = int(detection.xmin * width)
                y1 = int(detection.ymin * height)
                x2 = int(detection.xmax * width)
                y2 = int(detection.ymax * height)

                center_x = int((x1+x2)/2)
                center_y = int((y1+y2)/2)

                # === 绘制矩形或多边形 ===
                corners = np.array([[x1, y1], [x2, y1], [x2, y2], [x1, y2]], dtype=np.int32)
                cv2.polylines(frame, [corners], isClosed=True, color=(0, 255, 0), thickness=2)
                cv2.circle(frame, (center_x, center_y), 5, (0, 0, 255), -1)
                cv2.putText(frame, tag_info, (100, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


                t = TransformStamped()
                t.header.stamp = self.get_clock().now().to_msg()
                t.header.frame_id = "depth_camera_link" 
                # t.child_frame_id = f"object_{detection.label}"
                t.child_frame_id = f"object_1"
                t.transform.translation.x = det_msg.position.x
                t.transform.translation.y = det_msg.position.y-0.03
                t.transform.translation.z = det_msg.position.z+0.03                
                    
                # === ��̬���㣺ͨ�����߱ȹ��� yaw �Ƕ� ===
                # bbox_width = detection.xmax - detection.xmin
                # bbox_height = detection.ymax - detection.ymin
                # if bbox_height > 0:
                #     aspect_ratio = bbox_width / bbox_height
                # else:
                #     aspect_ratio = 1.0  # ��ֹ��0

                # if aspect_ratio > 1.5:
                #     estimated_yaw_deg = 90.0  # ���
                # elif aspect_ratio < 0.66:
                #     estimated_yaw_deg = 0.0   # ����
                # else:
                #     estimated_yaw_deg = 45.0  # ��б

                # import math
                # yaw = 3.1415926+math.radians(estimated_yaw_deg)
                # roll = 0.0
                # pitch = 1.571
                # R = Rscipy.from_euler('xyz', [roll, pitch, yaw]).as_matrix()
                # # 转换为四元数
                # rot = Rscipy.from_matrix(R)
                # qx, qy, qz, qw = rot.as_quat()

                # t.transform.rotation.x = qx
                # t.transform.rotation.y = qy
                # t.transform.rotation.z = qz
                # t.transform.rotation.w = qw
            
                self.tf_broadcaster.sendTransform(t)

                msg.detections.append(det_msg)

        self.detection_pub.publish(msg)
        gst_process.stdin.write(frame.tobytes())
        gst_process.stdin.flush()

    def destroy_node(self):
        super().destroy_node()
        cv2.destroyAllWindows()

def main(args=None):
    rclpy.init(args=args)
    node = OakYoloNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

