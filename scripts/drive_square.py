#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist


class DriveSquare(object):
    """ This node will drive forward for a specified time, turn 90 degrees, and then repeat effectively moving the robot in a square. """
    # Constructor/initializer
    def __init__(self):
        rospy.init_node('drive_in_square')
        self.msg = Twist()
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.turning = False

    # "Drives" the robot, causing it to move forward and the turn left, indefinitely.
    # To learn how to turn a robot 90 degrees, I found the following article useful: http://wiki.ros.org/turtlesim/Tutorials/Rotating%20Left%20and%20Right
    def run(self):
        # Define constants
        pi = 3.1415926535897
        
        angular_speed = 30 * 2 * pi/360
        relative_angle = 90 * 2 * pi/360
        
        forward_speed = 0.1
        forward_time = 5.0

        # Wait until time has begun ticking to avoid race condition
        while rospy.Time.now().to_sec() == 0.0:
            continue

        # Loop until robot shuts down
        while not rospy.is_shutdown():
            r = rospy.Rate(10)

            # Move forwards
            t0 = rospy.Time.now().to_sec()
            t1 = t0
            print("linear", t0)
            while (t1 - t0 < forward_time):
                self.msg.linear.x = forward_speed
                self.publisher.publish(self.msg)
                t1 = rospy.Time.now().to_sec()

                r.sleep()

            # Stop moving
            self.msg.linear.x = 0.0
            self.publisher.publish(self.msg)
            r.sleep()

            # Make a 90 degree left turn
            t0 = rospy.Time.now().to_sec()
            current_angle = 0
            print("angular", t0)
            while (current_angle < relative_angle):
                self.msg.angular.z = angular_speed
                self.publisher.publish(self.msg)
                t1 = rospy.Time.now().to_sec()
                current_angle = angular_speed * (t1 - t0)

                r.sleep()

            # Stop turning
            self.msg.angular.z = 0.0
            self.publisher.publish(self.msg)
            r.sleep()


if __name__ == '__main__':
    node = DriveSquare()
    node.run()