#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan


class PersonFollower(object):
    """ This node will approach the closest object and move towards it until it is quite close. """
    def __init__(self):
        rospy.init_node('drive_in_square')
        self.msg = Twist()
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subcriber = rospy.Subscriber('/scan', LaserScan, self.follow_person)

        # Proportional Control
        self.angular_constant = 0.02
        self.linear_constant = 0.5

    # Turns robot towards nearest object
    def follow_person(self, msg):
        # Find index and distance of closest object
        closest = 2^32
        index = 0

        for i in range(len(msg.ranges)):
            if msg.ranges[i] > 0 and msg.ranges[i] < closest:
                closest = msg.ranges[i]
                index = i

        # Do nothing if no objects in range
        if closest == 2^32:
            self.msg.linear.x = 0.0
            self.msg.angular.z = 0.0

            self.publisher.publish(self.msg)
            print("Linear:", self.msg.linear.x)
            return

        # We want midpoint to be 180 to make for easier proportional control
        index = (index + 180) % 360

        # Calculate errors (we want 180 degrees for angular, and 0.2m for linear)
        angular_error = 180 - index
        linear_error = 0.2 - closest
        angular_error *= -1
        linear_error *= -1

        # Calculate velocity - only change linear velocity if robot is more or less pointed at object
        if 150 <= index <= 210:
            self.msg.linear.x = self.linear_constant * linear_error
        else:
            self.msg.linear.x = 0.1
        self.msg.angular.z = self.angular_constant * angular_error

        print("Linear:", self.msg.linear.x)


        # Publish
        self.publisher.publish(self.msg)

    # Run the robot
    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = PersonFollower()
    node.run()