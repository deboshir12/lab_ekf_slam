#!/usr/bin/env python3
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from tf.transformations import euler_from_quaternion, quaternion_from_euler

class SimpleOdometryPublisher:
    def __init__(self):
        rospy.init_node('simple_odometry_publisher')
        self.odom_pub = rospy.Publisher('/odom', Odometry, queue_size=10)
        self.cmd_sub = rospy.Subscriber('/cmd_vel', Twist, self.cmd_callback)
        
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.rate = rospy.Rate(10)

    def cmd_callback(self, msg):
        # Simple forward movement
        dt = 0.1  # 10 Hz
        self.x += msg.linear.x * dt * np.cos(self.theta)
        self.y += msg.linear.x * dt * np.sin(self.theta)
        self.theta += msg.angular.z * dt

    def publish_odometry(self):
        # Publish the Odometry message
        odom = Odometry()
        odom.header.stamp = rospy.Time.now()
        odom.header.frame_id = 'odom'

        # Position
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0
        
        # Orientation (convert theta to quaternion)
        quaternion = quaternion_from_euler(0, 0, self.theta)
        odom.pose.pose.orientation.x = quaternion[0]
        odom.pose.pose.orientation.y = quaternion[1]
        odom.pose.pose.orientation.z = quaternion[2]
        odom.pose.pose.orientation.w = quaternion[3]

        # Twist (for robot velocity)
        odom.twist.twist.linear.x = 0.0
        odom.twist.twist.angular.z = 0.0

        # Publish the message
        self.odom_pub.publish(odom)

    def run(self):
        while not rospy.is_shutdown():
            self.publish_odometry()
            self.rate.sleep()

if __name__ == '__main__':
    try:
        odom_publisher = SimpleOdometryPublisher()
        odom_publisher.run()
    except rospy.ROSInterruptException:
        pass