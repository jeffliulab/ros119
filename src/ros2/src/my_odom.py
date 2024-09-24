#!/usr/bin/env python3

import rospy
import math
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Point 
from tf.transformations import euler_from_quaternion

class MyOdom:
    def __init__(self):
        self.odom_sub = rospy.Subscriber('odom', Odometry, self.odom_cb)
        self.my_odom_pub = rospy.Publisher('my_odom', Point, queue_size=1)
        self.old_pose = None 
        self.dist = 0.0
        self.yaw = 0.0
                
    def odom_cb(self, msg):
        """
        Callback function for `odom_sub`.

        Review: Odometry message (nav_msgs/Odometry) contains the robot's pose position, 
        orientation, and twist (including linear velocity and angular velocity).
        
        my_odom node receives Odometry messages published by TurtleBot3 through the odom topic
        """
        cur_pose = msg.pose.pose
        self.update_dist(cur_pose) 
        self.update_yaw(cur_pose.orientation)
        self.publish_data()

    def update_dist(self, cur_pose):
        """
        Helper to `odom_cb`.
        
        Updates `self.dist` to the distance between `self.old_pose` and `cur_pose`.

        This function receives the distance and then 
        calculates the cumulative moving distance based on the distance. 
        The unit here is meters.
        """

        if self.old_pose is not None:
            x_move = cur_pose.position.x - self.old_pose.x 
            y_move = cur_pose.position.y - self.old_pose.y 
            self.dist += math.sqrt(x_move**2 + y_move**2)

        self.old_pose = cur_pose.position # update old position

    def update_yaw(self, cur_orientation):
        """
        Helper to `odom_cb`.
        Updates `self.yaw` to current heading of robot.

        Review: Quaternion quaternion = w + xi + yj + zk, 
        the quaternion can be converted to Euler angle 
        using the conversion function provided by ROS
        """
        cur_quaternion = [cur_orientation.x, cur_orientation.y, cur_orientation.z, cur_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(cur_quaternion)
        self.yaw = yaw      # Update the current yaw angle

    def publish_data(self):
        """
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        # The `Point` object should be used simply as a data container for
        # `self.dist` and `self.yaw` so we can publish it on `my_odom`.
        p = Point()         # p is a Point message
        p.x = self.dist 
        p.y = self.yaw
        p.z = 0.0           # Point message has x,y,z three float64 variables
        self.my_odom_pub.publish(p)
        
        
if __name__ == '__main__':
    rospy.init_node('my_odom')
    MyOdom()
    rospy.spin()
