#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist



'''
PA2_Wall Follower Assignment Requirments

1. Function Goal: Follow the wall, maintain a fixed distance

2. Consider the following basic starting conditions:
(1) Robot already at the right distance from the wall and pointing parallel to the wall
(2) Robot at 1.5 the desired distance and pointing in the right direction
(3) Robot at 0.5 the desired distance and pointing in the right direction
These are some more challenging starting conditions:
(4) Robot is pointing in various other ways
(5) Robot is well away from the wall
(6) The wall has an inside corner
(7) The wall has an outside corner

3. Demo Represents Through:
(1) roslaunch turtlebot3_gazebo turtlebot3_stage_1.launch
(2) roslaunch turtlebot3_gazebo turtlebot3_gazebo_rviz.launch
(3) Put the robot where you want it to start
(4) rosrun wall_follower wall_follower.py
'''

class WallFollower:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        # 当订阅到新的激光雷达数据时，ROS 会自动调用 scan_cb() 回调函数，传递最新的激光雷达数据（msg）。
        self.scan_sub = rospy.Subscriber('/scan', LaserScan, self.scan_cb)

    def scan_cb(self, msg):
        ### Callback function for `self.scan_sub`.
        ### 这里我把scan脚本的内容融入进来，这样运行的时候就只需要这一个脚本运行就可以了
        ### scan_cb() 是处理激光雷达数据的地方，所有的距离测量、方向判断等逻辑都将在这里进行。



    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """
        raise NotImplementedError

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    WallFollower().follow_wall()






