#!/usr/bin/env python3

import rospy
import numpy as np
from geometry_msgs.msg import Pose, PoseArray
from nav_msgs.msg import PoseWithCovarianceStamped

class LandmarkDetector:
    def __init__(self):
        rospy.init_node('landmark_detector_node')

        # Known landmark positions (in map frame)
        self.landmarks = {
            0: np.array([5.0, 5.0]),
            1: np.array([2.0, 8.0])
        }

        # Subscribe to current pose from EKF or odometry
        rospy.Subscriber('/ekf_pose', PoseWithCovarianceStamped, self.pose_callback)  # Or use PoseWithCovarianceStamped

        # Publisher for landmark detections
        self.pub = rospy.Publisher('/landmark_detections', PoseArray, queue_size=10)

        self.timer = rospy.Timer(rospy.Duration(1.0), self.publish_landmarks)  # every 1 sec

        self.current_pose = np.zeros(3)  # x, y, theta
        rospy.spin()

    def pose_callback(self, msg):
        self.current_pose[0] = msg.pose.pose.position.x
        self.current_pose[1] = msg.pose.pose.position.y

        # Get orientation as yaw (simplified for 2D)
        # Assuming quaternion is only rotation in Z (flat ground)
        import tf.transformations
        quat = msg.pose.pose.orientation
        euler = tf.transformations.euler_from_quaternion([quat.x, quat.y, quat.z, quat.w])
        self.current_pose[2] = euler[2]  # theta

    def publish_landmarks(self, event):
        x, y, theta = self.current_pose
        msg = PoseArray()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "base_link"

        for landmark_id, (lx, ly) in self.landmarks.items():
            dx = lx - x
            dy = ly - y

            # Transform to robot frame
            rel_x = np.cos(theta)*dx + np.sin(theta)*dy
            rel_y = -np.sin(theta)*dx + np.cos(theta)*dy

            # Skip if out of sensor range (optional)
            if np.hypot(rel_x, rel_y) > 5.0:
                continue

            pose = Pose()
            pose.position.x = rel_x
            pose.position.y = rel_y
            pose.orientation.w = 1.0  # No rotation

            msg.poses.append(pose)

        self.pub.publish(msg)

if __name__ == '__main__':
    try:
        LandmarkDetector()
    except rospy.ROSInterruptException:
        pass