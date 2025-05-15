#!/usr/bin/env python3

import rospy
import numpy as np
from sensor_msgs.msg import LaserScan, Range
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped, PoseArray

class EKFLocalization:
    def __init__(self):
        rospy.init_node('ekf_localization_node')
        
        # Known landmarks in world frame (ID: [x, y])
        self.landmarks = {
            0: np.array([5.0, 5.0]),
            1: np.array([2.0, 8.0]),
            # Add more landmarks if needed
        }

        # State: [x, y, theta]
        self.state = np.zeros((3, 1))
        self.covariance = np.eye(3) * 0.01

        # Noise matrices
        self.Q = np.diag([0.05, 0.05, np.deg2rad(1)])  # Process noise
        self.R_ultra = np.array([[0.1]])               # Ultrasound measurement noise
        self.R_lidar = np.array([[0.5]])               # LIDAR measurement noise
        self.R_landmark = np.diag([0.3, 0.3])           # Landmark measurement noise

        # Subscribers
        rospy.Subscriber('/odom', Odometry, self.odom_callback)
        rospy.Subscriber('/ultrasound', Range, self.ultrasound_callback)
        rospy.Subscriber('/scan', LaserScan, self.lidar_callback)
        rospy.Subscriber('/landmark_detections', PoseArray, self.landmark_callback)

        # Publisher
        self.pose_pub = rospy.Publisher('/ekf_pose', PoseWithCovarianceStamped, queue_size=10)

        rospy.spin()

    def odom_callback(self, msg):
        dt = 0.1  # assuming 10 Hz
        vx = msg.twist.twist.linear.x
        vth = msg.twist.twist.angular.z

        theta = self.state[2, 0]
        dx = vx * np.cos(theta) * dt
        dy = vx * np.sin(theta) * dt
        dtheta = vth * dt

        delta_state = np.array([[dx], [dy], [dtheta]])
        self.state += delta_state

        Fx = np.eye(3)
        Fx[0, 2] = -vx * np.sin(theta) * dt
        Fx[1, 2] = vx * np.cos(theta) * dt
        self.covariance = Fx @ self.covariance @ Fx.T + self.Q

        self.publish_pose()

    def ultrasound_callback(self, msg):
        z = np.array([[msg.range]])
        expected_z = np.array([[2.0]])  # Dummy expected value

        H = np.array([[1, 0, 0]])
        self.ekf_update(z, expected_z, H, self.R_ultra)

    def lidar_callback(self, msg):
        angle_index = len(msg.ranges) // 2
        distance = msg.ranges[angle_index]
        z = np.array([[distance]])
        expected_z = np.array([[2.0]])

        H = np.array([[1, 0, 0]])
        self.ekf_update(z, expected_z, H, self.R_lidar)

    def landmark_callback(self, msg):
        x, y, theta = self.state.flatten()

        for i, pose in enumerate(msg.poses):
            if i not in self.landmarks:
                continue

            # Measurement from robot's perspective
            z = np.array([[pose.position.x], [pose.position.y]])

            # Predicted landmark position from current state
            landmark_pos = self.landmarks[i]
            dx = landmark_pos[0] - x
            dy = landmark_pos[1] - y

            # Expected measurement in robot frame
            expected_z = np.array([
                [np.cos(theta)*dx + np.sin(theta)*dy],
                [-np.sin(theta)*dx + np.cos(theta)*dy]
            ])

            # Jacobian H of the measurement function
            q = dx**2 + dy**2
            H = np.array([
                [-np.cos(theta), -np.sin(theta),  dy],
                [ np.sin(theta), -np.cos(theta), -dx]
            ])

            self.ekf_update(z, expected_z, H, self.R_landmark)

    def ekf_update(self, z, expected_z, H, R):
        y = z - expected_z
        S = H @ self.covariance @ H.T + R
        K = self.covariance @ H.T @ np.linalg.inv(S)

        self.state = self.state + K @ y
        self.covariance = (np.eye(3) - K @ H) @ self.covariance

    def publish_pose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = 'map'
        msg.pose.pose.position.x = self.state[0, 0]
        msg.pose.pose.position.y = self.state[1, 0]
        msg.pose.pose.orientation.w = 1.0  # Simplified

        # Flatten covariance matrix to 36 elements
        cov = np.zeros((6, 6))
        cov[:3, :3] = self.covariance
        msg.pose.covariance = cov.flatten().tolist()
        self.pose_pub.publish(msg)

if __name__ == '__main__':
    try:
        EKFLocalization()
    except rospy.ROSInterruptException:
        pass