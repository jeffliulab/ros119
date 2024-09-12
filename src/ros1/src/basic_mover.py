#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion

# BasicMover
class BasicMover:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        
        # 因为遇到了bug，所以这里修改constructor，初始化值也依赖my_odom推送过来的数据
        # Current distance moved and heading of the robot.
        initial_odom = rospy.wait_for_message('my_odom', Point)
        self.start_distance = initial_odom.x
        self.moved_distance = initial_odom.x 
        self.cur_yaw = initial_odom.y

    def my_odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        # my_odom Node计算完成后，会发送 Point message回来（回到basic_mover这儿来）
        # 复习：Point message（geometry_msgs/Point）包括x, y, z三个浮点数，可以用来表示机器人的位置
        self.moved_distance = msg.x # moved distance, received from my_odom topic
        self.cur_yaw = msg.y # yaw, received from my_odom topic


    def turn_to_heading(self, target_yaw):
        """
        Turns the robot to heading `target_yaw`.
        """
        # 这个函数的目的是让机器人从现在的yaw旋转到目标yaw
        twist = Twist()
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            delta_yaw = target_yaw - self.cur_yaw

            # 打印调试信息，确保 delta_yaw 正常更新 ###########
            rospy.loginfo(f"Delta yaw: {delta_yaw:.3f}, Current Yaw: {self.cur_yaw:.3f}, Target Yaw: {target_yaw:.3f}")


            #如果误差角度很小，停止旋转
            if abs(delta_yaw) < 0.05:
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                #########################################
                rospy.loginfo("旋转完毕 ROTATION COMPLETE")
                break 
            
            # 根据偏差方向旋转
            twist.angular.z = 0.3 if delta_yaw >0 else -0.3
            self.cmd_vel_pub.publish(twist)
            rate.sleep() 
        
        #### 提升点：这里可以改成动态调整角速度

    def move_forward(self, target_dist):
        """Moves the robot forward by `target_dist`."""
        twist = Twist()
        rate = rospy.Rate(10)

        self.start_distance = self.moved_distance

        twist.linear.x = 0.1
        while not rospy.is_shutdown():
            # 计算当前距离相对于起点的变化
            delta_distance = self.moved_distance - self.start_distance    
            
            # 打印调试信息，确保 delta_distance 正常更新 ###########
            rospy.loginfo(f"Delta Distance: {delta_distance:.3f} of {target_dist:.3f}")

            # 如果移动的距离达到目标，则停止移动
            if delta_distance >= target_dist:
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo("到达目标距离 REACHED TARGET DISTANCE")
                break 
            # 否则继续移动
            self.cmd_vel_pub.publish(twist)
            rate.sleep()
    
            


    def out_and_back(self, target_dist):
        """
        This function:
        1. moves the robot forward by `target_dist`;
        2. turns the robot by 180 degrees; and
        3. moves the robot forward by `target_dist`.
        """
        # 这个函数实现作业中要求的前进1m，调转车头，然后后退1m
        self.move_forward(target_dist)

        target_yaw = self.cur_yaw + math.pi # math.pi means 180 degrees，在ROS中，角度使用的是弧度制
        if target_yaw > math.pi:
            target_yaw -= 2 * math.pi #......
        
        self.turn_to_heading(target_yaw)
        self.move_forward(target_dist)
        



    def draw_square(self, side_length):
        """
        This function moves the robot in a square with `side_length` meter sides.
        """ 
        # 这个函数实现作业中要求的画方块
        raise NotImplementedError

    def move_in_a_circle(self, r):
        """Moves the robot in a circle with radius `r`"""
        # 这个函数实现作业中的画圈圈
        raise NotImplementedError
        
    def rotate_in_place(self):
        """For debugging."""
        twist = Twist()
        
        rate = rospy.Rate(10)

        while not rospy.is_shutdown():
            twist.angular.z = 0.1
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

if __name__ == '__main__':
    rospy.init_node('basic_mover')
    BasicMover().out_and_back(1)
    # BasicMover().draw_square(1)
    # BasicMover().move_in_a_circle(1)
    # BasicMover().rotate_in_place()
