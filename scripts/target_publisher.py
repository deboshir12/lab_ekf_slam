#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Point

def send_goal(x, y):
    rospy.init_node('target_sender')
    pub = rospy.Publisher('/target_point', Point, queue_size=1)
    rospy.sleep(1)  # Wait for the publisher to register

    goal = Point()
    goal.x = x
    goal.y = y
    pub.publish(goal)
    rospy.loginfo(f"Target ({x}, {y}) sent.")

def input_target():
    rospy.loginfo("Ready to receive target coordinates.")
    while not rospy.is_shutdown():
        # Input loop to get coordinates from user
        try:
            x = float(input("Enter target X coordinate: "))
            y = float(input("Enter target Y coordinate: "))
        except ValueError:
            rospy.logwarn("Invalid input. Please enter numeric values for coordinates.")
            continue
        
        # Send the goal after receiving valid input
        send_goal(x, y)

if __name__ == '__main__':
    try:
        input_target()  # Set goal manually or from args
    except rospy.ROSInterruptException:
        pass