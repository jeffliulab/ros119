#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion

'''
版本v1中仍面临的问题：
1、依靠度数而非pi的旋转无法实现
2、无法正确判断转向右还是转向左（可能会出现需要左转90但实际右转270的问题）
'''



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
            # 朝着正确的方向旋转
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
            twist.angular.z = 0.3 if delta_yaw >0 else -0.3 #角速度
            self.cmd_vel_pub.publish(twist)
            rate.sleep() 
        
        ###### 提升点：这里可以改成动态调整角速度

    def move_forward(self, target_dist):
        """Moves the robot forward by `target_dist`."""
        twist = Twist()
        rate = rospy.Rate(10)

        self.start_distance = self.moved_distance

        twist.linear.x = 0.3 # 前进的线速度

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

    ''' 旋转上总是出问题，暂时先不用了
    # 上面的旋转部分改为函数，方便使用
    def rotate_degree(self, target_degree):
        angle_in_radians = math.radians(target_degree) # 用度数记录
        
        # 计算目标偏航角
        target_yaw = self.cur_yaw + angle_in_radians

        # 向正确的方向旋转
        target_yaw = self.right_rotate(target_yaw)

        # 执行旋转
        self.turn_to_heading(target_yaw)

    # 这里有一个问题，就是旋转的时候虽然角度最终是对的，但是老是反着旋转
    def right_rotate(self,angle):
        # 这里实践的时候还有个问题，就是经常反着旋转
        while angle > math.pi:
            angle -= 2 * math.pi
        while angle < -math.pi:
            angle += 2 * math.pi
        return angle
    

    def draw_square(self, side_length):
        """
        This function moves the robot in a square with `side_length` meter sides.
        """ 
        # 这个函数实现作业中要求的画方块
        # 其实就是在上面的基础上，前进1m=》右拐=〉前进1m=》右拐。。。。
        for _ in range(4):  # 正方形有四条边
            self.move_forward(side_length)
            self.rotate_degree(90)  # 每次旋转90度


    def move_in_a_circle(self, r):
        """Moves the robot in a circle with radius `r`"""
        # 这个函数实现作业中的画圈圈
        # 根据机器人运动学的公式，当机器人以恒定的角速度和线速度行驶时，它会沿着一个圆形轨迹运动。
        num_segments = 36  # 将圆分为36段（每次前进一个小弧段）
        angle_per_segment = 360 / num_segments  # 每段旋转的角度 (度数)

        # 每段的弧长 s = r * θ (θ 以弧度计算)
        distance_per_segment = (2 * math.pi * r) / num_segments  # 每段直线前进的距离

        # 逐段执行：前进一小段，然后转动一定角度
        for _ in range(num_segments):
            self.move_forward(distance_per_segment)  # 前进一小段距离
            self.rotate_degree(angle_per_segment)    # 旋转一定的角度

    '''

    def draw_square(self, side_length):
        """
        This function moves the robot in a square with `side_length` meter sides.
        """ 
        # 这个函数实现作业中要求的画方块
        # 其实就是在上面的基础上，前进1m=》右拐=〉前进1m=》右拐。。。。
        for _ in range(4):  # 正方形有四条边
            self.move_forward(side_length)
            # 旋转90度 (π/2 弧度)
            target_yaw = self.cur_yaw + math.pi / 2
            if target_yaw > math.pi:
                target_yaw -= 2 * math.pi
            self.turn_to_heading(target_yaw)


    def move_in_a_circle(self, r):
        """Moves the robot in a circle with radius `r`"""
        # 这个函数实现作业中的画圈圈
        # 根据机器人运动学的公式，当机器人以恒定的角速度和线速度行驶时，它会沿着一个圆形轨迹运动。
        """
        让机器人按照半径 r 画一个圆形轨迹，并在画完一圈后停止。
        """
        twist = Twist()
        rate = rospy.Rate(10)  # 控制循环频率
        
        # 根据半径 r 和设定的线速度计算所需的角速度
        linear_speed = 0.2  # 线速度 (单位：m/s)
        angular_speed = linear_speed / r  # 根据线速度和半径计算角速度
        
        twist.linear.x = linear_speed  # 设置线速度
        twist.angular.z = angular_speed  # 设置角速度

        # 计算一圈所需的时间
        circumference = 2 * math.pi * r  # 圆的周长
        total_time = circumference / linear_speed  # 走完一圈所需的总时间

        rospy.loginfo(f"Starting to move in a circle with radius {r} meters, will take {total_time:.2f} seconds to complete the circle.")
        
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed_time = (rospy.Time.now() - start_time).to_sec()  # 计算经过的时间

            # 如果经过的时间超过了总时间，则停止
            if elapsed_time >= total_time:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)  # 停止机器人
                rospy.loginfo("Completed the circle and stopped.")
                break

            # 否则继续沿着圆形轨迹运动
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

        
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
    rospy.sleep(3) #让机器人等待3秒钟
    BasicMover().draw_square(1)
    rospy.sleep(3)
    BasicMover().move_in_a_circle(1)
    # BasicMover().rotate_in_place()
