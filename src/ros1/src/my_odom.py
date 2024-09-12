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
        """Callback function for `odom_sub`."""
        # 复习：Odometry message (nav_msgs/Odometry)包含机器人的pose位置、orientation方向、以及twist（包含线速度和角速度）
        # my_odom node通过odom topic接收TurtleBot3发布的Odometry message
        cur_pose = msg.pose.pose
        self.update_dist(cur_pose) 
        self.update_yaw(cur_pose.orientation)
        self.publish_data()

    def update_dist(self, cur_pose):
        """
        Helper to `odom_cb`.
        Updates `self.dist` to the distance between `self.old_pose` and
        `cur_pose`.
        """
        #######################################################
        # 这个函数接收距离，然后根据距离计算累计的移动距离，这里的单位是米
        if self.old_pose is not None:
            x_move = cur_pose.position.x - self.old_pose.x 
            y_move = cur_pose.position.y - self.old_pose.y 
            self.dist += math.sqrt(x_move**2 + y_move**2)

        self.old_pose = cur_pose.position # update old position

    def update_yaw(self, cur_orientation):
        """
        Helper to `odom_cb`.
        Updates `self.yaw` to current heading of robot.
        """
        #########################################################
        # 四元数quaternion = w + xi + yj + zk， 用ROS自带的转换函数可以将四元数转换为欧拉角
        cur_quaternion = [cur_orientation.x, cur_orientation.y, cur_orientation.z, cur_orientation.w]
        (roll, pitch, yaw) = euler_from_quaternion(cur_quaternion)
        self.yaw = yaw # 更新当前的偏航角

    def publish_data(self):
        """
        Publish `self.dist` and `self.yaw` on the `my_odom` topic.
        """
        # The `Point` object should be used simply as a data container for
        # `self.dist` and `self.yaw` so we can publish it on `my_odom`.
        p = Point() # p is a Point message
        p.x = self.dist 
        p.y = self.yaw
        p.z = 0.0 # Point message has x,y,z three float64 variables
        self.my_odom_pub.publish(p)
        
        
if __name__ == '__main__':
    rospy.init_node('my_odom')
    MyOdom()
    rospy.spin()
