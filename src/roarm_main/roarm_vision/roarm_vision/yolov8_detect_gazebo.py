import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from depthai_ros_msgs.msg import SpatialDetectionArray, SpatialDetection
from vision_msgs.msg import BoundingBox2D, ObjectHypothesis,Pose2D, Point2D
from geometry_msgs.msg import Point,TransformStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
from ultralytics import YOLO
import numpy as np
from sensor_msgs.msg import Image, CameraInfo
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
import tf_transformations

class Yolov8Detect(Node):
    def __init__(self):
        super().__init__('yolov8_detect')

        self.camera_info_sub = self.create_subscription(CameraInfo, '/oak/camera_info', self.camera_info_callback, 10)
        self.rgb_sub = self.create_subscription(Image, '/oak/image_raw', self.rgb_callback, 10)
        self.depth_sub = self.create_subscription(Image, '/oak/depth/image_raw', self.depth_callback, 10)

        self.detection_pub = self.create_publisher(SpatialDetectionArray, '/spatial_detections', 10)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        # self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_broadcaster = TransformBroadcaster(self, qos=rclpy.qos.QoSProfile(depth=10))

        self.camera_frame = "depth_camera_link"  
        self.base_frame = "base_link" 

        self.bridge = CvBridge()
        self.yolo = YOLO("model.pt")  
        self.depth_image = None 
        self.camera_info = None

    def camera_info_callback(self, msg: CameraInfo):
        self.camera_info = msg
        
    def pixel_to_real_world(self, x, y, depth):
        fx = self.camera_info.k[0]  
        fy = self.camera_info.k[4]  
        cx = self.camera_info.k[2]  
        cy = self.camera_info.k[5]  

        X = (x - cx) * depth / fx
        Y = (y - cy) * depth / fy
        Z = depth
        return X, Y, Z

    def depth_callback(self, msg):
        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            self.get_logger().error(f"error: {e}")

    def rgb_callback(self, msg):
        try:
            rgb_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            results = self.yolo(rgb_image)

            if self.depth_image is None:
                self.get_logger().warn("未收到深度图像，跳过深度计算")
                return
            
            depth_h, depth_w = self.depth_image.shape
            detection_msg = SpatialDetectionArray()
            detection_msg.header.stamp = self.get_clock().now().to_msg()
            detection_msg.header.frame_id = "3d_camera_link"

            for result in results:
                for box in result.boxes:
                    x1, y1, x2, y2 = map(int, box.xyxy[0].tolist())

                    x1, y1 = max(0, x1), max(0, y1)
                    x2, y2 = min(depth_w, x2), min(depth_h, y2)

                    new_x1 = x1 + (x2 - x1) // 4
                    new_y1 = y1 + (y2 - y1) // 4
                    new_x2 = x2 - (x2 - x1) // 4
                    new_y2 = y2 - (y2 - y1) // 4

                    new_x1, new_y1 = max(0, new_x1), max(0, new_y1)
                    new_x2, new_y2 = min(depth_w, new_x2), min(depth_h, new_y2)

                    roi = self.depth_image[new_y1:new_y2, new_x1:new_x2]
                    valid_depths = roi[roi > 0]  

                    avg_depth = np.mean(valid_depths) if valid_depths.size > 0 else -1.0

                    detection = SpatialDetection()
                    obj_hypothesis = ObjectHypothesis()
                    obj_hypothesis.class_id = str(int(box.cls[0].item()))
                    obj_hypothesis.score = float(box.conf[0].item())
                    detection.results.append(obj_hypothesis)

                    detection.bbox.center.position = Point2D(x=(x1 + x2) / 2.0, y=(y1 + y2) / 2.0)
                    detection.bbox.center.theta = 0.0  
                    detection.bbox.size_x = float(x2 - x1)  
                    detection.bbox.size_y = float(y2 - y1) 

                    real_x, real_y, real_z = self.pixel_to_real_world(
                        (x1 + x2) / 2, (y1 + y2) / 2, avg_depth)
                    
                    detection.position = Point(x=float(real_x), 
                                               y=float(real_y), 
                                               z=float(real_z))

                    detection.is_tracking = False
                    detection.tracking_id = ""


                    try:
                        transform = self.tf_buffer.lookup_transform(self.base_frame, self.camera_frame, rclpy.time.Time())
                        trans = transform.transform.translation
                        rot = transform.transform.rotation

                        cam_pose = np.array([detection.position.x, detection.position.y, detection.position.z, 1.0])

                        q = [rot.x, rot.y, rot.z, rot.w]
                        R = tf_transformations.quaternion_matrix(q)[:3, :3]
                        T = np.array([trans.x, trans.y, trans.z])

                        base_pose = R @ cam_pose[:3] + T

                        detection.position.x = base_pose[0]
                        detection.position.y = base_pose[1]
                        detection.position.z = base_pose[2]

                        t = TransformStamped()
                        t.header.stamp = self.get_clock().now().to_msg()
                        t.header.frame_id = self.base_frame
                        t.child_frame_id = f"object_{obj_hypothesis.class_id}"
                        t.transform.translation.x = base_pose[0]
                        t.transform.translation.y = base_pose[1]
                        t.transform.translation.z = base_pose[2]
                        t.transform.rotation = rot  

                        self.tf_broadcaster.sendTransform(t)

                    except Exception as e:
                        self.get_logger().warn(f"TF transform failed: {str(e)}")

                    detection_msg.detections.append(detection)

            self.detection_pub.publish(detection_msg)

        except Exception as e:
            self.get_logger().error(f"RGB 处理失败: {e}")
            
        except Exception as e:
            self.get_logger().error(f"RGB 处理失败: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = Yolov8Detect()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

