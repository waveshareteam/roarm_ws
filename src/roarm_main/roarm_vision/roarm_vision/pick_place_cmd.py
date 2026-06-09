import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String,Header
from cv_bridge import CvBridge
from geometry_msgs.msg import Point,TransformStamped
from builtin_interfaces.msg import Time
from tf2_ros import Buffer, TransformListener, TransformBroadcaster
from scipy.spatial.transform import Rotation as Rscipy
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult
from std_msgs.msg import Float32
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from rclpy.duration import Duration
from geometry_msgs.msg import Pose
from tf2_ros import TransformException
from roarm_msgs.srv import PickPlaceCmd
from scipy.spatial.transform import Rotation as R

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
from .roarm_solver import RoArmM2, RoArmM3

roarm_model = os.environ['ROARM_MODEL']
gripper_type = os.environ['GRIPPER_TYPE']
if roarm_model == 'roarm_m2':
    roarm = RoArmM2()
elif roarm_model == 'roarm_m3':
    roarm = RoArmM3()

class GripperPublisherNode(Node):

    def __init__(self):
        super().__init__("gripper_publisher_node")

        self.gripper_pub = self.create_publisher(
            Float32,
            "/gripper_cmd",
            10
        )

    def publish_gripper_cmd(self, value: float):
        msg = Float32()
        msg.data = value

        self.gripper_pub.publish(msg)

        self.get_logger().info(
            f"Published gripper command: {value}"
        )

class HandPublisherNode(Node):

    class TrajPoint:
        def __init__(self, joints, duration, max_speed):
            self.joints = joints
            self.duration = duration
            self.max_speed = max_speed

    def __init__(self):
        super().__init__("hand_publisher_node")

        self.hand_pub = self.create_publisher(
            JointTrajectory,
            "/hand_controller/joint_trajectory",
            10
        )

        self.traj_points = []

        if roarm_model  == "roarm_m2":
            self.last_point = [0.0, 0.0, 2.618]

        elif roarm_model  == "roarm_m3":
            self.last_point = [0.0, 0.0, 2.618, -1.0472, 0.0]

        self.get_logger().info(
            f"HandPublisherNode initialized with roarm_model : {roarm_model}"
        )

    def add_point(self, joint_positions, duration=1.5, max_speed=0.5):

        self.traj_points.append(
            self.TrajPoint(joint_positions, duration, max_speed)
        )

        self.get_logger().info(
            f"Added point #{len(self.traj_points)} "
            f"(t={duration:.2f}s, v={max_speed:.2f}m/s)"
        )

    def clear_points(self):
        self.traj_points = []

    def publish_trajectory(self):

        if len(self.traj_points) == 0:
            self.get_logger().warn("No points to publish!")
            return

        self.get_logger().info(
            "Only one point, adding last_point as start."
        )

        self.traj_points.insert(
            0,
            self.TrajPoint(self.last_point, 0.0,
                           self.traj_points[0].max_speed)
        )

        traj = JointTrajectory()

        traj.header.stamp = (
            self.get_clock().now()
            + Duration(seconds=0.1)
        ).to_msg()

        if roarm_model  == "roarm_m2":
            traj.joint_names = [
                "base_link_to_link1",
                "link1_to_link2",
                "link2_to_link3"
            ]

        elif roarm_model  == "roarm_m3":
            traj.joint_names = [
                "base_link_to_link1",
                "link1_to_link2",
                "link2_to_link3",
                "link3_to_link4",
                "link4_to_link5"
            ]

        t_cumulative = 0.0

        for seg in range(len(self.traj_points) - 1):

            p0 = self.traj_points[seg]
            p1 = self.traj_points[seg + 1]

            if len(p0.joints) != len(p1.joints):

                self.get_logger().error(
                    f"Segment #{seg} joint dimension mismatch"
                )
                return

            dist = 0.0

            for j in range(len(p0.joints)):
                dist += (p1.joints[j] - p0.joints[j]) ** 2

            dist = math.sqrt(dist)

            if dist < 1e-6:

                self.get_logger().warn(
                    f"Segment #{seg} skipped (distance too small)"
                )
                continue

            if p1.duration > 0:
                dt = p1.duration
            else:
                dt = dist / p1.max_speed

            for j in range(len(p0.joints)):

                delta = abs(p1.joints[j] - p0.joints[j])
                v = delta / dt

                if v > p1.max_speed:

                    scale = v / p1.max_speed
                    dt *= scale

                    self.get_logger().warn(
                        f"Segment #{seg} joint[{j}] exceeds speed "
                        f"({v:.3f}>{p1.max_speed:.3f}), "
                        f"increasing duration to {dt:.3f}s"
                    )

            if dt < 1e-3:

                self.get_logger().warn(
                    f"Segment #{seg} duration too small ({dt}), clamped"
                )

                dt = 1e-3

            steps = max(10, int(dt * 10))

            self.get_logger().info(
                f"Segment #{seg}: dist={dist:.4f}, "
                f"duration={dt:.3f}s, steps={steps}"
            )

            for i in range(steps + 1):

                t = dt * i / steps
                tau = np.clip(t / dt, 0.0, 1.0)

                s = (
                    10 * tau**3
                    - 15 * tau**4
                    + 6 * tau**5
                )

                point = JointTrajectoryPoint()

                point.positions = [0.0] * len(p0.joints)
                point.velocities = [0.0] * len(p0.joints)

                for j in range(len(p0.joints)):

                    point.positions[j] = (
                        p0.joints[j]
                        + s * (p1.joints[j] - p0.joints[j])
                    )

                    ds_dt = (
                        30 * tau**2
                        - 60 * tau**3
                        + 30 * tau**4
                    ) / dt

                    point.velocities[j] = (
                        ds_dt * (p1.joints[j] - p0.joints[j])
                    )

                    if (
                        math.isnan(point.positions[j])
                        or math.isinf(point.positions[j])
                        or math.isnan(point.velocities[j])
                        or math.isinf(point.velocities[j])
                    ):
                        self.get_logger().error(
                            f"Invalid joint value detected "
                            f"at segment #{seg}, joint[{j}]"
                        )
                        return

                point.time_from_start = Duration(
                    seconds=t_cumulative + t + 1e-4 * seg
                ).to_msg()

                traj.points.append(point)

            t_cumulative += dt

        if len(traj.points) == 0:

            self.get_logger().warn(
                "No valid segments to publish!"
            )
            return

        self.last_point = self.traj_points[-1].joints

        self.hand_pub.publish(traj)

        self.get_logger().info(
            f"Published trajectory with "
            f"{len(traj.points)} samples, total {t_cumulative:.2f}s"
        )

        self.clear_points()

class TargetPoseSubscription(Node):

    def __init__(self):

        super().__init__("pick_place_cmd")

        self.tf_buffer = Buffer(cache_time=Duration(seconds=0.6))
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.target_pose = Pose()

    def update_target_pose(self,target_frame,base_frame,cam_frame):

        try:

            transform1 = self.tf_buffer.lookup_transform(
                base_frame,
                target_frame,
                rclpy.time.Time()
            )

            transform2 = self.tf_buffer.lookup_transform(
                cam_frame,
                target_frame,
                rclpy.time.Time()
            )

            self.target_pose.position.x = transform1.transform.translation.x
            self.target_pose.position.y = transform1.transform.translation.y
            self.target_pose.position.z = transform1.transform.translation.z

            self.target_pose.orientation = transform2.transform.rotation

        except TransformException as ex:

            self.get_logger().warn(
                f"Could not transform {base_frame} to {target_frame}: {ex}"
            )
            return None

        return self.target_pose

class SimplePID:
    def __init__(self, kp=0.5, ki=0.0, kd=0.1):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0.0
        self.prev_err = 0.0

    def update(self, error):
        self.integral += error
        self.integral = max(-100, min(100, self.integral))  # anti-windup
        derivative = error - self.prev_err
        self.prev_err = error
        return self.kp * error + self.ki * self.integral + self.kd * derivative

    def reset(self):
        self.integral = 0.0
        self.prev_err = 0.0

def limit_yaw(yaw):
    v = yaw

    while v > math.pi:
        v -= 2 * math.pi

    while v < -math.pi:
        v += 2 * math.pi

    if abs(abs(v) - math.pi) < 0.3:
        v = 0.0

    return v

class PickPlaceCmdNode(Node):
    def __init__(self, hand_node, gripper_node, targetPose_node):
        super().__init__('pick_place_cmd')
        self.hand_node = hand_node
        self.gripper_node = gripper_node
        self.targetPose_node = targetPose_node

        self.declare_parameter('base_frame', 'ugv_roarm_base_link')
        self.declare_parameter('cam_frame', 'camera_link')
        self.base_frame = self.get_parameter('base_frame').value
        self.cam_frame = self.get_parameter('cam_frame').value

        self.srv = self.create_service(
            PickPlaceCmd,
            'pick_place_cmd',
            self.handle_pick_place
        )

        self.get_logger().info("PickPlaceService started. Waiting for pick/place commands...")

        self.current_xyz = [0.0, 0.0, 0.0]

    def handle_pick_place(self, request, response):

        if request.cmd == 0:

            success = self.move(request.target)

            response.success = success

        if request.cmd == 1:

            success = self.pick(request.target,request.gripper)

            response.success = success

        if request.cmd == 2:

            success = self.place()

            response.success = success

        return response

    def get_current_position_mm(self):
        return self.current_xyz

    def visual_servo_tf(self, target_frame, timeout=10.0, 
                        tolerance_mm=5.0, dead_zone_mm=3.0, diff=20.0):
        pid_x = SimplePID(kp=0.8, ki=0.0, kd=0.00)
        pid_y = SimplePID(kp=0.7, ki=0.0, kd=0.05)

        start = time.time()
        ok_count = 0

        while time.time() - start < timeout:
            pose_obj = self.targetPose_node.update_target_pose(
                target_frame, self.base_frame, self.cam_frame)
            if pose_obj is None:
                time.sleep(0.1)
                continue

            obj_x = pose_obj.position.x * 1000
            obj_y = pose_obj.position.y * 1000 + diff

            grip_x, grip_y, grip_z = self.get_current_position_mm()

            if gripper_type == 'angular_gear':
                err_x = obj_x - grip_x
                err_y = obj_y - grip_y - diff
            else:
                err_x = obj_x - grip_x
                err_y = obj_y - grip_y

            err_dist = math.sqrt(err_x**2 + err_y**2)

            self.get_logger().info(f"servo err: x={err_x:.1f} y={err_y:.1f} mm")

            if err_dist < tolerance_mm and abs(err_x) < tolerance_mm  and abs(err_y) < tolerance_mm:
                ok_count += 1
                if ok_count >= 3:
                    self.get_logger().info("Aligned!")
                    return True
                time.sleep(0.1)
                continue
            else:
                ok_count = 0

            if abs(err_x) < dead_zone_mm:
                err_x = 0.0
            if abs(err_y) < dead_zone_mm:
                err_y = 0.0

            dx = pid_x.update(err_x)
            dy = pid_y.update(err_y)

            if abs(dx) < 0.5 and abs(dy) < 0.5:
                time.sleep(0.1)
                continue

            angles = roarm.compute_joint_rad_by_pos(
                grip_x + dx,
                grip_y + dy,
                grip_z,
                0.0
            )
            if angles is None:
                continue
            self.current_xyz = [grip_x + dx, grip_y + dy, grip_z]
            self.hand_node.add_point(angles, 1.0)
            self.hand_node.publish_trajectory()
            time.sleep(1.0)

        self.get_logger().warn("Servo timeout")
        return False

    def standoff_xyz(self, x_mm, y_mm, z_mm, d_mm=100.0):
        r = math.hypot(x_mm, y_mm)
        if r < 1e-3:
            return x_mm - d_mm, y_mm, z_mm
        return (
            x_mm - d_mm * x_mm / r,
            y_mm - d_mm * y_mm / r,
            z_mm,
        )

    def move(self,target):
        self.get_logger().info("Start Move")

        target_frame=f"object_{target}"

        if roarm_model =='roarm_m2':
            home=[0.0, 0.0, 2.618]
        elif roarm_model =='roarm_m3':
            home=[0.0, 0.0, 1.5708, 1.5708, 0.0]

        self.hand_node.add_point(home)
        self.hand_node.publish_trajectory()
        time.sleep(1.5)

        self.gripper_node.publish_gripper_cmd(1.5)
        # time.sleep(3)
            
        pose = self.targetPose_node.update_target_pose(
                target_frame,
                self.base_frame,
                self.cam_frame)

        if pose is None:
            self.get_logger().warn("Pose is None")
            return False

        if pose.position.x == 0.0 and pose.position.y == 0.0 and pose.position.z == 0.0:
            self.get_logger().warn("Pose is zero, skip")
            return False

        x = pose.position.x * 1000
        y = pose.position.y * 1000 + 40
        z = pose.position.z * 1000

        x, y, z = self.standoff_xyz(x, y, z, d_mm=100.0)

        if roarm_model =='roarm_m2':
            if gripper_type == 'angular_gear':
                angles_first = roarm.compute_joint_rad_by_pos(x, y, z, gripper)
            else:
                angles_first = roarm.compute_joint_rad_by_pos(x, y, z, 0)
        elif roarm_model =='roarm_m3':
            rot = R.from_quat([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ])

            roll, pitch, yaw = rot.as_euler('xyz')
            self.get_logger().info(f"roll: {roll}")
            self.get_logger().info(f"pitch: {pitch}")
            self.get_logger().info(f"yaw: {yaw}")
            pitch_fixed = limit_yaw(pitch)
            roll_fixed = 1.571
            angles_first = roarm.compute_joint_rad_by_pos(
                x, y, z,
                pitch_fixed,
                roll_fixed,
                0.0
            )

        if any(math.isnan(a) for a in angles_first):
            self.get_logger().warn("IK failed")
            return False

        self.hand_node.add_point(angles_first, 3.0)
        self.hand_node.publish_trajectory()
        time.sleep(3)

        pose = self.targetPose_node.update_target_pose(
                target_frame,
                self.base_frame,
                self.cam_frame)
                
        if pose is None:
            self.get_logger().warn("Pose is None")
            self.hand_node.add_point(home)
            self.hand_node.publish_trajectory()
            time.sleep(1.5)
            return False
            
        x = pose.position.x * 1000
        y = pose.position.y * 1000 + 30
        z = pose.position.z * 1000 - 87.459 + 50

        if roarm_model =='roarm_m2':
            if gripper_type == 'angular_gear':
                angles_second = roarm.compute_joint_rad_by_pos(x, y, z, gripper)
            else:
                angles_second = roarm.compute_joint_rad_by_pos(x, y, z, 0)
        elif roarm_model =='roarm_m3':
            rot = R.from_quat([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ])

            roll, pitch, yaw = rot.as_euler('xyz')
            self.get_logger().info(f"roll: {roll}")
            self.get_logger().info(f"pitch: {pitch}")
            self.get_logger().info(f"yaw: {yaw}")
            pitch_fixed = limit_yaw(pitch)
            roll_fixed = 1.571
            angles_second = roarm.compute_joint_rad_by_pos(
                x, y, z,
                pitch_fixed,
                roll_fixed,
                0.0
            )

        if any(math.isnan(a) for a in angles_second):
            self.get_logger().warn("IK failed")
            return False

        self.hand_node.add_point(angles_second,1.0)
        self.hand_node.publish_trajectory()
        time.sleep(1.0)

        self.current_xyz = [x, y, z]

        aligned = self.visual_servo_tf(target_frame, timeout=10.0, tolerance_mm=4.0,diff=30.0)
        if not aligned:
            self.hand_node.add_point(home)
            self.hand_node.publish_trajectory()
            return False

        ax, ay, az = self.get_current_position_mm()
        if roarm_model =='roarm_m2':
            angles = roarm.compute_joint_rad_by_pos(ax, ay, az, 0.0)         
        elif roarm_model =='roarm_m3':
            rot = R.from_quat([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ])
            roll, pitch, yaw = rot.as_euler('xyz')
            self.get_logger().info(f"roll: {roll}")
            self.get_logger().info(f"pitch: {pitch}")
            self.get_logger().info(f"yaw: {yaw}")
            pitch_fixed = limit_yaw(pitch)
            roll_fixed = 1.571
            angles = roarm.compute_joint_rad_by_pos(
                ax, ay, az-40,
                pitch_fixed,
                roll_fixed,
                0.0
            )
            
        if any(math.isnan(a) for a in angles):
            self.get_logger().warn("IK failed")
            return False

        self.hand_node.add_point(angles,1.0)
        self.hand_node.publish_trajectory()
        time.sleep(1.0)

        self.hand_node.add_point(angles_second,1.0)
        self.hand_node.publish_trajectory()
        time.sleep(1.0)

        self.hand_node.add_point(home)
        self.hand_node.publish_trajectory()
        time.sleep(1.5)

        self.get_logger().info("Pick finished")

        return True

    def pick(self,target,gripper):
        self.get_logger().info("Start Pick")

        target_frame=f"object_{target}"

        if roarm_model =='roarm_m2':
            home=[0.0, 0.0, 2.618]
        elif roarm_model =='roarm_m3':
            home=[0.0, 0.0, 1.5708, 1.5708, 0.0]

        self.hand_node.add_point(home)
        self.hand_node.publish_trajectory()
        time.sleep(1.5)

        self.gripper_node.publish_gripper_cmd(1.5)
        # time.sleep(3)
            
        pose = self.targetPose_node.update_target_pose(
                target_frame,
                self.base_frame,
                self.cam_frame)

        if pose is None:
            self.get_logger().warn("Pose is None")
            return False

        if pose.position.x == 0.0 and pose.position.y == 0.0 and pose.position.z == 0.0:
            self.get_logger().warn("Pose is zero, skip")
            return False

        x = pose.position.x * 1000
        y = pose.position.y * 1000 + 30
        z = pose.position.z * 1000

        x, y, z = self.standoff_xyz(x, y, z, d_mm=100.0)

        if roarm_model =='roarm_m2':
            if gripper_type == 'angular_gear':
                angles_first = roarm.compute_joint_rad_by_pos(x, y, z, gripper)
            else:
                angles_first = roarm.compute_joint_rad_by_pos(x, y, z, 0)
        elif roarm_model =='roarm_m3':
            rot = R.from_quat([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ])

            roll, pitch, yaw = rot.as_euler('xyz')
            self.get_logger().info(f"roll: {roll}")
            self.get_logger().info(f"pitch: {pitch}")
            self.get_logger().info(f"yaw: {yaw}")
            pitch_fixed = limit_yaw(pitch)
            roll_fixed = 1.571
            angles_first = roarm.compute_joint_rad_by_pos(
                x, y, z,
                pitch_fixed,
                roll_fixed,
                0.0
            )

        if any(math.isnan(a) for a in angles_first):
            self.get_logger().warn("IK failed")
            return False

        self.hand_node.add_point(angles_first, 3.0)
        self.hand_node.publish_trajectory()
        time.sleep(3)

        pose = self.targetPose_node.update_target_pose(
                target_frame,
                self.base_frame,
                self.cam_frame)
                
        if pose is None:
            self.get_logger().warn("Pose is None")
            self.hand_node.add_point(home)
            self.hand_node.publish_trajectory()
            time.sleep(1.5)
            return False
            
        x = pose.position.x * 1000
        y = pose.position.y * 1000 + 20
        z = pose.position.z * 1000 - 87.459 + 50

        if roarm_model =='roarm_m2':
            if gripper_type == 'angular_gear':
                angles_second = roarm.compute_joint_rad_by_pos(x, y, z, gripper)
            else:
                angles_second = roarm.compute_joint_rad_by_pos(x, y, z, 0)
        elif roarm_model =='roarm_m3':
            rot = R.from_quat([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ])

            roll, pitch, yaw = rot.as_euler('xyz')
            self.get_logger().info(f"roll: {roll}")
            self.get_logger().info(f"pitch: {pitch}")
            self.get_logger().info(f"yaw: {yaw}")
            pitch_fixed = limit_yaw(pitch)
            roll_fixed = 1.571
            angles_second = roarm.compute_joint_rad_by_pos(
                x, y, z,
                pitch_fixed,
                roll_fixed,
                0.0
            )

        if any(math.isnan(a) for a in angles_second):
            self.get_logger().warn("IK failed")
            return False

        self.hand_node.add_point(angles_second,1.0)
        self.hand_node.publish_trajectory()
        time.sleep(1.0)

        self.current_xyz = [x, y, z]

        aligned = self.visual_servo_tf(target_frame, timeout=10.0, tolerance_mm=4.0,diff=20.0)
        if not aligned:
            self.hand_node.add_point(home)
            self.hand_node.publish_trajectory()
            return False

        ax, ay, az = self.get_current_position_mm()
        if roarm_model =='roarm_m2':
            if gripper_type == 'angular_gear':
                angles = roarm.compute_joint_rad_by_pos(ax, ay, az-60, gripper)
            else:
                angles = roarm.compute_joint_rad_by_pos(ax, ay, az-60, 0)   
        elif roarm_model =='roarm_m3':
            rot = R.from_quat([
                pose.orientation.x,
                pose.orientation.y,
                pose.orientation.z,
                pose.orientation.w
            ])
            roll, pitch, yaw = rot.as_euler('xyz')
            self.get_logger().info(f"roll: {roll}")
            self.get_logger().info(f"pitch: {pitch}")
            self.get_logger().info(f"yaw: {yaw}")
            pitch_fixed = limit_yaw(pitch)
            roll_fixed = 1.571
            angles = roarm.compute_joint_rad_by_pos(
                ax, ay, az-40,
                pitch_fixed,
                roll_fixed,
                0.0
            )
            
        if any(math.isnan(a) for a in angles):
            self.get_logger().warn("IK failed")
            return False

        self.hand_node.add_point(angles,1.0)
        self.hand_node.publish_trajectory()
        time.sleep(1.0)

        self.gripper_node.publish_gripper_cmd(gripper)
        time.sleep(1.5)

        self.hand_node.add_point(angles_second,1.0)
        self.hand_node.publish_trajectory()
        time.sleep(1.0)

        self.hand_node.add_point(home)
        self.hand_node.publish_trajectory()
        time.sleep(1.5)

        self.get_logger().info("Pick finished")

        return True

    def place(self):

        self.get_logger().info("Start Place")

        if roarm_model =='roarm_m2':
            home=[1.5708, 0.0, 2.618]
        elif roarm_model =='roarm_m3':
            home=[1.5708, 0.0, 2.618, 0.0, 0.0]

        self.hand_node.add_point(home)
        self.hand_node.publish_trajectory()
        time.sleep(3)

        self.gripper_node.publish_gripper_cmd(1.5)
        time.sleep(1.5)

        self.gripper_node.publish_gripper_cmd(0.0)

        if roarm_model =='roarm_m2':
            back=[0.0, 0.0, 2.618]
        elif roarm_model =='roarm_m3':
            back=[0.0, 0.0, 2.618, 0.0, 0.0]

        self.hand_node.add_point(back)
        self.hand_node.publish_trajectory()
        time.sleep(3)

        self.get_logger().info("Place finished")

        return True

def main(args=None):
    # Initialize the ROS client library
    rclpy.init(args=args)
    hand_node = HandPublisherNode()
    gripper_node = GripperPublisherNode()
    targetPose_node = TargetPoseSubscription()
    pick_place_cmd_node = PickPlaceCmdNode(
        hand_node,
        gripper_node,
        targetPose_node,
    )
    executor = rclpy.executors.MultiThreadedExecutor()

    executor.add_node(pick_place_cmd_node)
    executor.add_node(hand_node)
    executor.add_node(gripper_node)
    executor.add_node(targetPose_node)

    executor.spin()
    # Shutdown the ROS client library
    rclpy.shutdown()