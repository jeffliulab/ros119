#!/usr/bin/env python3
import math
import rospy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import Twist



'''
PA2_Wall Follower Assignment Requirments

1. Function Goal: Follow the wall, maintain a fixed distance

2. Consider the following basic starting conditions:

现在就剩这两种情况没有讨论了：

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
        
        self.infinite_distance = 10.0 # 假设最大距离是10.0

        self.dead_distance = 0.2 # 设定的离墙太近的距离（因为雷达不够精确的问题，此处需要特殊逻辑）
        self.keep_distance = 0.5 # 设定的相对距离（也就是要保持的距离）
        self.boundry_distance = 3.0 # 设定的边界距离（也就是well away的定义）

        self.angular_speed = 0.3 # 角速度，正代表逆时针旋转，也就是向左转
        self.angular_speed_slow = self.angular_speed * 0.3 # 在死区的缓慢旋转速度

        self.linear_speed = 0.25 # 线速度
        self.linear_speed_slow = self.linear_speed * 0.3 # 在墙壁处、边界处的缓慢移动速度

        self.closest_obstacle_dist = self.infinite_distance # 最近的距离
        self.closest_direction = 0                          # 最近的距离的方向的角度
        self.foreward_distance = self.infinite_distance 
        self.left_distance = self.infinite_distance 
        self.backward_distance = self.infinite_distance 
        self.right_distance = self.infinite_distance 


    
    def scan_cb(self, msg):
        ### 回调函数的参数是由消息类型决定的，所以这里不能直接加一个n作为参数，来获得方向n的距离
        ### Callback function for `self.scan_sub`.
        ### 这里我把scan脚本的内容融入进来，这样运行的时候就只需要这一个脚本运行就可以了
        ### scan_cb() 是处理激光雷达数据的地方，所有的距离测量、方向判断等逻辑都将在这里进行。
        ### 每当 /scan 话题发布新的激光雷达数据时，ROS 会把数据作为参数 msg 传递到这个函数。
        ### msg 是 LaserScan 消息对象，包含了激光雷达扫描的所有数据信息，比如：
        ### ranges[]：数组，包含激光雷达每个方向上测量到的距离数据。
        ### angle_min 和 angle_max：表示扫描角度范围（弧度）。
        ### angle_increment：表示每两个扫描点之间的角度差。

        ### 最近距离: closest_obstacle_dist
        valid_closest_dist = min([r for r in msg.ranges if r != float('inf')]) if [r for r in msg.ranges if r != float('inf')] else self.infinite_distance
        # valid_closest_dist = min([r for r in msg.ranges if r != float('inf')]) else self.infinite_distance
        self.closest_obstacle_dist = valid_closest_dist if valid_closest_dist != float('inf') else self.infinite_distance
        rospy.loginfo(f"Closest obstacle distance: {self.closest_obstacle_dist}") # 打印调试信息

        ### 最近距离所代表的方向: closest_direction
        if self.closest_obstacle_dist < self.infinite_distance:
            self.closest_direction = msg.ranges.index(self.closest_obstacle_dist) 
            rospy.loginfo(f"Closest distance direction: {self.closest_direction}") # 打印调试信息
        else:
            self.closest_direction = -1 # -1 means there is no closest direction


        self.foreward_distance = self.get_distance_at_direction(msg, 0) # 0 是正前方: foreward_distance
        self.left_distance = self.get_distance_at_direction(msg, 90) # 90 是正左方: left_distance
        self.backward_distance = self.get_distance_at_direction(msg, 180) # 180 是正后方: backward_distance
        self.right_distance = self.get_distance_at_direction(msg, 270) # 270 是正右方: right_distance

    def get_distance_at_direction(self, msg, n):
        """
        获取激光雷达扫描的 n 方向的距离
        """
        return msg.ranges[n] if msg.ranges[n] != float('inf') else self.infinite_distance

    def follow_wall(self):
        """
        Makes the robot follow a wall.
        """
        twist = Twist()
        
        ### 这里可以简化代码，设置函数内的变量来简化

        if self.closest_obstacle_dist < self.dead_distance:
            ### (6) The wall has an inside corner
            # 死区逻辑
            # 逃离死区：用robot的前后左右，而不是用最近距离

            # 如果机器人左手方向距离足够大，则逆时针旋转到左手方向，然后前进3s，直到前进到安全距离内
            if self.left_distance > self.dead_distance:
                time_to_rotate = math.pi / 2 # 逆时针旋转90度，即逆时针旋转pi/2弧度
                start_time = rospy.Time.now().to_sec()
                while rospy.Time.now().to_sec() - start_time < time_to_rotate:
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed_slow
                    rospy.sleep(0.1)  # 适当休眠，避免过度发布消息
                start_time = rospy.Time.now().to_sec()  #前进3s
                while rospy.Time.now().to_sec() - start_time < 3:
                    twist.angular.z = 0.0
                    twist.linear.x = self.linear_speed_slow     
                    rospy.sleep(0.1)
                

            # 如果右手方向距离足够大，则顺时针旋转到右手方向，然后前进3s，直到前进到安全距离内
            elif self.right_distance > self.dead_distance:
                time_to_rotate = - math.pi / 2 # 顺时针旋转90度
                start_time = rospy.Time.now().to_sec()
                while rospy.Time.now().to_sec() - start_time < time_to_rotate:
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed_slow
                    rospy.sleep(0.1)  # 适当休眠，避免过度发布消息
                start_time = rospy.Time.now().to_sec()  #前进3s
                while rospy.Time.now().to_sec() - start_time < 3:
                    twist.angular.z = 0.0
                    twist.linear.x = self.linear_speed_slow     
                    rospy.sleep(0.1)

            # 如果backward距离足够大，或者其他的情况，一律旋转180度，然后前进3s
            else :
                time_to_rotate = math.pi  # 逆时针旋转180度，即逆时针旋转pi弧度
                start_time = rospy.Time.now().to_sec()
                while rospy.Time.now().to_sec() - start_time < time_to_rotate:
                    twist.linear.x = 0.0
                    twist.angular.z = self.angular_speed_slow
                    rospy.sleep(0.1)  # 适当休眠，避免过度发布消息
                start_time = rospy.Time.now().to_sec()
                while rospy.Time.now().to_sec() - start_time < 3:
                    twist.angular.z = 0.0
                    twist.linear.x = self.linear_speed_slow     
                    rospy.sleep(0.1)




        elif self.closest_obstacle_dist < self.keep_distance:
            ### (1) Robot already at the right distance from the wall and pointing parallel to the wall
            ### (3) Robot at 0.5 the desired distance and pointing in the right direction
            ### (7) The wall has an outside corner --- the virtual wall can be assumed, so no problem
            # if 0 <= self.closest_direction < 30 or 330 <= self.closest_direction:
            #     # 死区 350～30度？？？
            #     # 这里应该怎么做？——慢慢旋转，慢慢前进！
            #     twist.angular.z = - self.angular_speed_slow
            #     twist.linear.x = self.linear_speed_slow
            if 0 <= self.closest_direction < 90:
                twist.angular.z = - self.angular_speed
                twist.linear.x = self.linear_speed_slow
            elif 270 <= self.closest_direction:
                twist.angular.z = self.angular_speed
                twist.linear.x = self.linear_speed_slow
            ### (4) Robot is pointing in various other ways (inside target distance, face backward the wall)
            # Keep going, no turn
            elif 90 <= self.closest_direction < 135 or 225 <= self.closest_direction < 270:
                twist.angular.z = 0.0
                twist.linear.x = self.linear_speed
            # Keep turning, no move
            elif 135 <= self.closest_direction < 180:
                twist.angular.z = self.angular_speed
                twist.linear.x = self.linear_speed_slow
            elif 180 <= self.closest_direction < 225:
                twist.angular.z = - self.angular_speed
                twist.linear.x = self.linear_speed_slow
        elif self.boundry_distance > self.closest_obstacle_dist > self.keep_distance:
            ### (2) Robot at 1.5 the desired distance and pointing in the right direction
            twist.linear.x = self.linear_speed
            if 90 <= self.closest_direction < 180:
                twist.angular.z = self.angular_speed
            elif 180<= self.closest_direction < 270:
                twist.angular.z = - self.angular_speed
            ### (4) Robot is pointing in various other ways (outside target distance, face toward the wall)
            # Keep going, no turn
            elif 45 <= self.closest_direction < 90 or 270 <= self.closest_direction < 315:
                twist.angular.z = 0.0
                twist.linear.x = self.linear_speed
            # Keep rotating, no move
            elif 0 <= self.closest_direction < 45:
                twist.angular.z = - self.angular_speed
                twist.linear.x = 0.0
            elif 315 <= self.closest_direction:
                twist.angular.z = self.angular_speed
                twist.linear.x = 0.0              
        elif self.closest_direction == -1 or self.closest_obstacle_dist >= self.boundry_distance - 0.1:  # 0.1 is the dead zone avoid chaos
            ### (5) Robot is well away from the wall
            # if there is no wall near the robot, draw big circle
            multi = 1 # Find the wall, can move faster
            r = 5 # the moving radius
            circular_direction = 1 # the moving direction
            
            circular_linear_speed = self.linear_speed * multi
            circular_angular_speed = circular_linear_speed / r
            twist.angular.z = circular_angular_speed * circular_direction
            twist.linear.x = circular_linear_speed

        self.cmd_vel_pub.publish(twist)

if __name__ == '__main__':
    rospy.init_node('wall_follower')
    
    wallfollower = WallFollower()

    rate = rospy.Rate(10)

    while not rospy.is_shutdown():
        wallfollower.follow_wall()
        rate.sleep()






