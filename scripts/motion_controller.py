#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
from sensor_msgs.msg import LaserScan
import math

class SimpleMotionController:
    def __init__(self):
        rospy.init_node('motion_controller_node')

        # Parameters
        self.goal = None
        self.current_pose = None
        self.obstacle_threshold = 0.5  # meters
        self.linear_speed = 0.2
        self.angular_speed = 1.0
        self.angle_tolerance = 0.1

        # Publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribers
        rospy.Subscriber('/target_point', Point, self.goal_callback)
        rospy.Subscriber('/ekf_pose', PoseWithCovarianceStamped, self.pose_callback)
        rospy.Subscriber('/scan', LaserScan, self.scan_callback)

        self.front_clear = True

        rospy.Timer(rospy.Duration(0.1), self.update)

        rospy.spin()

    def goal_callback(self, msg):
        self.goal = msg
        rospy.loginfo(f"New goal received: ({msg.x:.2f}, {msg.y:.2f})")

    def pose_callback(self, msg):
        self.current_pose = msg.pose.pose

    def scan_callback(self, msg):
        # Check the front sector (±10 degrees)
        ranges = msg.ranges
        front = ranges[len(ranges)//2 - 10 : len(ranges)//2 + 10]
        self.front_clear = all(r > self.obstacle_threshold or math.isinf(r) for r in front)

    def update(self, event):
        if self.goal is None or self.current_pose is None:
            return

        # Current pose
        x = self.current_pose.position.x
        y = self.current_pose.position.y
        # Orientation (only yaw from quaternion)
        q = self.current_pose.orientation
        yaw = self.quaternion_to_yaw(q)

        # Target direction
        dx = self.goal.x - x
        dy = self.goal.y - y
        target_dist = math.hypot(dx, dy)
        target_angle = math.atan2(dy, dx)
        angle_diff = self.normalize_angle(target_angle - yaw)

        cmd = Twist()

        if target_dist < 0.1:
            # Close enough to the goal
            cmd.linear.x = 0.0
            cmd.angular.z = 0.0
        elif abs(angle_diff) > self.angle_tolerance:
            # Rotate toward target
            cmd.angular.z = self.angular_speed * (1 if angle_diff > 0 else -1)
        elif self.front_clear:
            # Move forward
            cmd.linear.x = self.linear_speed
        else:
            # Obstacle detected: stop
            rospy.logwarn("Obstacle ahead! Stopping.")
            cmd.linear.x = 0.0

        self.cmd_pub.publish(cmd)

    @staticmethod
    def quaternion_to_yaw(q):
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny, cosy)

    @staticmethod
    def normalize_angle(angle):
        while angle > math.pi:
            angle -= 2*math.pi
        while angle < -math.pi:
            angle += 2*math.pi
        return angle

if __name__ == '__main__':
    try:
        SimpleMotionController()
    except rospy.ROSInterruptException:
        pass