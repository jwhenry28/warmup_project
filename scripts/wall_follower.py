#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan

class WallFollower(object):
    """ This node travels along a wall indefinitely, following the wall if it curves or turns. """
    def __init__(self):
        rospy.init_node('wall_follower')
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.subcriber = rospy.Subscriber('/scan', LaserScan, self.follow_wall)

        self.msg = Twist()

        # Proportional Control
        self.side_constant = 1.5
        self.directional_constant = 0.5
        self.linear_slow = 0.5
        
    def follow_wall(self, msg):
        # Keep moving straight if not close to wall
        closest = min(msg.ranges)
        if closest > 0.75:
            print("Driving to wall")
            self.msg.linear.x = 0.25
            self.msg.angular.z = 0.0
            self.publisher.publish(self.msg)
            return

        # We care about the front, immediate left side, and 45-degree angles on left side
        front_left_distance = msg.ranges[45]
        rear_left_distance = msg.ranges[135]
        side_distance = msg.ranges[90]
        front_distance = msg.ranges[0]

        print("Front:", round(front_distance, 5), "Front_Left:", round(front_left_distance, 5), "Side:", round(side_distance, 5), "Rear_Left:", round(rear_left_distance, 5))

        # Calculate direction as to always be parallel to wall and 0.25m from wall
        directional_error = front_left_distance - rear_left_distance
        side_error = side_distance - 0.25

        # Slow down as you approach a wall or corner, and start turning
        # We use the front distance to detect walls/corners, and the side_distance to ensure that the robot doesn't get "stuck" while turning in a corner
        if front_distance < 0.5 or (side_distance > front_left_distance and side_distance > rear_left_distance):
            self.msg.linear.x = min(0.25, max(0.05, self.msg.linear.x * self.linear_slow))
            self.msg.angular.z = -0.5
        # Otherwise keep moving, adjusting angular velocity to stay by wall.
        else:
            self.msg.linear.x = 0.25
            self.msg.angular.z = (self.side_constant * side_error) + (directional_error * self.directional_constant)

        print("directional_error:", round(directional_error, 5), "side_error", round(side_error, 5), "angular:", self.msg.angular.z)
        print("")

        self.publisher.publish(self.msg)

    def run(self):
        rospy.spin()


if __name__ == '__main__':
    node = WallFollower()
    node.run()
