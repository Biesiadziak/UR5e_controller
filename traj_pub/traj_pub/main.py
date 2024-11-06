#!/usr/bin/env python3
import rclpy 
from rclpy.node import Node
from sensor_msgs.msg import Image 
from cv_bridge import CvBridge 
import cv2 
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class movementNode(Node):
    def __init__(self):
        super().__init__("movement_node")
        self.window_name = "camera"
        self.camera_subscriber_ = self.create_subscription(Image, 'image_raw', self.camera_callback, 10)
        self.get_logger().info("camera node has started")
        self.velocity_publisher_ = self.create_publisher(JointTrajectory, '/scaled_joint_trajectory_controller/joint_trajectory', 10)
        self.traj = JointTrajectory()
        self.traj.joint_names = ["shoulder_lift_joint", "elbow_joint", "wrist_1_joint", "wrist_2_joint", "wrist_3_joint", "shoulder_pan_joint"]

    def camera_callback(self, image_data):
        self.cv_image = CvBridge().imgmsg_to_cv2(image_data,"bgr8")
        self.gray = cv2.cvtColor(self.cv_image, cv2.COLOR_BGR2GRAY)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
        self.parameters = cv2.aruco.DetectorParameters()

        self.detector = cv2.aruco.ArucoDetector(self.aruco_dict, self.parameters)
        self.corners, self.ids, self.rejected = self.detector.detectMarkers(self.cv_image)
        if self.ids is not None:
            self.sum_x = 0
            self.sum_y = 0
            for point in self.corners:
                for i in range(4):
                    self.sum_x += (point[0][i][0])  # współrzędna X
                    self.sum_y += (point[0][i][1])  # współrzędna X
            self.middle_point = [self.sum_x/4.0, self.sum_y/4.0]

            if self.middle_point[1] < 240.0:
                self.point = JointTrajectoryPoint()
                self.point.positions = [-1.5817583242999476, -2.128230873738424, -2.375350300465719, -1.2659714857684534, 3.920382022857666, 3.324960708618164]
                self.point.time_from_start = Duration(sec = 3)
            else:
                self.point = JointTrajectoryPoint()
                self.point.positions = [-1.5817583242999476, -2.128230873738424, -2.375350300465719, -0.7, 3.920382022857666, 3.324960708618164]
                self.point.time_from_start = Duration(sec = 3)

            self.traj.points = [self.point]
            self.velocity_publisher_.publish(self.traj)
        
        cv2.aruco.drawDetectedMarkers(self.cv_image, self.corners, self.ids)
        cv2.imshow('Detected Markers', self.cv_image)
        cv2.waitKey(25)
            


def main(args=None):
    rclpy.init(args=args)
    node = movementNode()
    rclpy.spin(node)
    rclpy.shutdown()