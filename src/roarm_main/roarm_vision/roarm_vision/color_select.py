import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class LabTrackbarNode(Node):
    def __init__(self):
        super().__init__('lab_trackbar_node')
        self.declare_parameter('image_topic', '/image_raw')
        image_topic = self.get_parameter('image_topic').value
        self.image_raw_subscription = self.create_subscription(Image,image_topic, self.image_callback,10)
        
        self.bridge = CvBridge()
        self.frame = None

        cv2.namedWindow("Lab Trackbars")
        cv2.createTrackbar("L Lower", "Lab Trackbars", 0, 255, lambda x: None)
        cv2.createTrackbar("L Upper", "Lab Trackbars", 255, 255, lambda x: None)
        cv2.createTrackbar("a Lower", "Lab Trackbars", 170, 255, lambda x: None)
        cv2.createTrackbar("a Upper", "Lab Trackbars", 255, 255, lambda x: None)
        cv2.createTrackbar("b Lower", "Lab Trackbars", 170, 255, lambda x: None)
        cv2.createTrackbar("b Upper", "Lab Trackbars", 255, 255, lambda x: None)

        self.create_timer(0.03, self.update_display)  
        self._exit_requested = False

    def image_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")

    def update_display(self):
        if self.frame is None:
            return

        lab = cv2.cvtColor(self.frame, cv2.COLOR_BGR2LAB)

        lL = cv2.getTrackbarPos("L Lower", "Lab Trackbars")
        lU = cv2.getTrackbarPos("L Upper", "Lab Trackbars")
        aL = cv2.getTrackbarPos("a Lower", "Lab Trackbars")
        aU = cv2.getTrackbarPos("a Upper", "Lab Trackbars")
        bL = cv2.getTrackbarPos("b Lower", "Lab Trackbars")
        bU = cv2.getTrackbarPos("b Upper", "Lab Trackbars")

        lower = np.array([lL, aL, bL])
        upper = np.array([lU, aU, bU])

        mask = cv2.inRange(lab, lower, upper)
        result = cv2.bitwise_and(self.frame, self.frame, mask=mask)

        cv2.imshow("Original", self.frame)
        cv2.imshow("Mask", mask)
        cv2.imshow("Result", result)
        key = cv2.waitKey(1)
        if key == 27:
            self.get_logger().info("ESC pressed, shutting down...")
            cv2.destroyAllWindows()
            print("Lower LAB:", lower)
            print("Upper LAB:", upper)
            self._exit_requested = True

def main(args=None):
    rclpy.init(args=args)
    node = LabTrackbarNode()
    try:
        while rclpy.ok() and not node._exit_requested:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()
