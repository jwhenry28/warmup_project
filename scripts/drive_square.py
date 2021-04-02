#!/usr/bin/env python3

import rospy
import time
from geometry_msgs.msg import Twist

"""
STATUS

Robot drives in a squareish pattern; the noise really messes it up though. 
"""


class DriveSquare(object):
    """ This node will drive forward for a specified time, turn 90 degrees, and then repeat effectively moving the robot in a square. """
    def __init__(self):
        rospy.init_node('drive_in_square')
        self.msg = Twist()
        self.publisher = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        self.turning = False

        # self.msg.linear.x = 0.25

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

        # base_time = time.time()
        # forward_duration = 5.0
        # turn_duration = 2.25
        # pause_duration1 = 0.5
        # pause_duration2 = 0.5

        # while not rospy.is_shutdown():
        #     r = rospy.Rate(10)

        #     now = time.time()
            
        #     # After pause, reset timer and resume moving forward
        #     if (now - base_time) >= (forward_duration + pause_duration1 + turn_duration + pause_duration2) and self.turning:
        #         print("Beginning forward")
        #         self.msg.linear.x = 0.1
        #         self.msg.angular.z = 0.0
        #         self.turning = False

        #         base_time = time.time()
            
        #     # Turn for turn_duration seconds and pause
        #     elif (now - base_time) >= (forward_duration + pause_duration1 + turn_duration) and self.turning:
        #         print("Pausing after turn")
        #         self.msg.linear.x = 0.0
        #         self.msg.angular.z = 0.0

        #     # Start turn after pause
        #     elif (now - base_time) >= (forward_duration + pause_duration1) and not self.turning:
        #         print("Beginning turn")
        #         self.msg.linear.x = 0.0
        #         self.msg.angular.z = 0.75
        #         self.turning = True

        #     # Drive forward forward_duration seconds and pause
        #     elif (now - base_time) >= (forward_duration) and not self.turning:
        #         print("Pausing after moving")
        #         self.msg.linear.x = 0.0
        #         self.msg.angular.z = 0.0

        #     # Keep turning
        #     elif self.turning:
        #         print("turning...")
        #         self.msg.linear.x = 0.0
        #         self.msg.angular.z = 0.75

        #     # Keep moving forward
        #     else:
        #         print("moving...")
        #         self.msg.linear.x = 0.1
        #         self.msg.angular.z = 0.0

        #     self.publisher.publish(self.msg)
            
        #     r.sleep()


if __name__ == '__main__':
    node = DriveSquare()
    node.run()