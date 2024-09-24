#!/usr/bin/env python3
import math
import rospy
from geometry_msgs.msg import Point, Pose, Twist
from tf.transformations import euler_from_quaternion
import numpy as np
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


'''
版本v2和v1没有关系，主要针对Sep17的课程中关于沿着墙壁的内容进行了修改
并在simulation house map中进行了试验
'''

'''
这个代码是课程中提供的代码，用来计算PID的（沿着墙壁跑的那个）
'''
class PID:
    def __init__(self, min_val, max_val, kp, ki, kd):
        self.min_val = min_val
        self.max_val = max_val
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.integral = 0
        self.prev_error = 0

    def compute(self, setpoint, measured_value):
        error = setpoint - measured_value
        self.integral += error
        self.derivative = error - self.prev_error
        if setpoint == 0 and self.prev_error == 0:
            self.integral = 0
        pid = self.kp * error + self.ki * self.integral + self.kd * self.derivative
        self.prev_error = error
        return np.clip(pid, self.min_val, self.max_val)

class BasicMover:
    def __init__(self):
        self.cmd_vel_pub = rospy.Publisher('cmd_vel', Twist, queue_size=1)
        self.my_odom_sub = rospy.Subscriber('my_odom', Point, self.my_odom_cb)
        self.my_laser = rospy.Subscriber('/obstacle_distance', Float32, self.my_laser_cb)
        
        # Current distance moved and heading of the robot, depends on odom
        initial_odom = rospy.wait_for_message('my_odom', Point)
        self.start_distance = initial_odom.x
        self.moved_distance = initial_odom.x 
        self.cur_yaw = initial_odom.y

        # 初始化障碍物距离，等待第一条激光数据来初始化min_distance
        self.min_distance = rospy.wait_for_message('/obstacle_distance', Float32).data


    def my_laser_cb(self, msg):
        self.min_distance = msg.data

    def my_odom_cb(self, msg):
        """Callback function for `self.my_odom_sub`."""
        # After my_odom Node completes the calculation, it will send a Point message to basic_mover
        self.moved_distance = msg.x     # moved distance, received from my_odom topic
        self.cur_yaw = msg.y            # yaw, received from my_odom topic


    def turn_to_heading(self, target_yaw):
        """
        Turns the robot to heading `target_yaw`.
        """
        # The purpose of this function is to rotate the robot from the current yaw to the target yaw        
        twist = Twist()
        rate = rospy.Rate(10)

        # 【PID】初始化PID控制器
        pid_controller = PID(min_val=-0.5, max_val=0.5, kp=0.5, ki=0.01, kd=0.1)


        while not rospy.is_shutdown():
            # Rotate in the right direction
            delta_yaw = target_yaw - self.cur_yaw

            # 【PID】使用PID控制角速度
            angular_speed = pid_controller.compute(target_yaw, self.cur_yaw)


            # Print debug information to ensure delta_yaw is updated normally
            rospy.loginfo(f"Delta yaw: {delta_yaw:.3f}, Current Yaw: {self.cur_yaw:.3f}, Target Yaw: {target_yaw:.3f}")

            # If the delta angle is very small, stop rotating
            if abs(delta_yaw) < 0.05:
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo("ROTATION COMPLETE")
                break 
            
            # Rotate according to deviation direction
            twist.angular.z = angular_speed # 【PID】使用PID控制角速度
            self.cmd_vel_pub.publish(twist)
            rate.sleep() 
        
    def move_forward(self, target_dist):
        """Moves the robot forward by `target_dist`."""
        twist = Twist()
        rate = rospy.Rate(10)

        self.start_distance = self.moved_distance

        # 【PID】初始化PID控制器
        pid_controller = PID(min_val=0.0, max_val=0.2, kp=1.0, ki=0.02, kd=0.01)


        while not rospy.is_shutdown():
            # Calculate the change in current distance relative to the starting point
            delta_distance = self.moved_distance - self.start_distance    
            
            # Print debug information to ensure delta_distance is updated normally
            rospy.loginfo(f"Delta Distance: {delta_distance:.3f} of {target_dist:.3f}")

            # 【PID】使用PID控制线速度
            linear_speed = pid_controller.compute(target_dist, self.moved_distance - self.start_distance)

            # If the moving distance reaches the target, stop moving
            if delta_distance >= target_dist:
                twist.linear.x = 0.0
                self.cmd_vel_pub.publish(twist)
                rospy.loginfo("到达目标距离 REACHED TARGET DISTANCE")
                break 
            # Otherwise keep moving
            twist.linear.x = linear_speed
            self.cmd_vel_pub.publish(twist)
            rate.sleep()

    def out_and_back(self, target_dist):
        """
        This function:
        1. moves the robot forward by `target_dist`;
        2. turns the robot by 180 degrees; and
        3. moves the robot forward by `target_dist`.
        """
        self.move_forward(target_dist)

        # In ROS, angles are measured in radians. math.pi means 180 degrees.
        target_yaw = self.cur_yaw + math.pi
        if target_yaw > math.pi:
            target_yaw -= 2 * math.pi
        self.turn_to_heading(target_yaw)
        
        self.move_forward(target_dist)

    def draw_square(self, side_length):
        """
        This function moves the robot in a square with `side_length` meter sides.
        """ 
        
        # Actually, it is just like: forward 1m => turn right => forward 1m => turn right. . . .
        for _ in range(4):  # A square has four sides
            self.move_forward(side_length)
            target_yaw = self.cur_yaw + math.pi / 2  # Rotate 90 degrees (π/2 radians)
            if target_yaw > math.pi:
                target_yaw -= 2 * math.pi
            self.turn_to_heading(target_yaw)


    def move_in_a_circle(self, r):
        """
        Moves the robot in a circle with radius `r`
        According to the formula of robot kinematics, 
        when the robot moves at a constant angular velocity and linear velocity, 
        it moves along a circular trajectory.
        """
        twist = Twist()
        rate = rospy.Rate(10)
        
        # Calculate the required angular velocity based on the radius r and the set linear velocity
        linear_speed = 0.2  # Linear speed (unit: m/s)
        angular_speed = linear_speed / r  # Calculate angular velocity from linear velocity and radius
        
        twist.linear.x = linear_speed  # Set line speed
        twist.angular.z = angular_speed  # Set the angular velocity

        # Calculate the time required for one lap
        circumference = 2 * math.pi * r  # Circumference of a circle
        total_time = circumference / linear_speed  # Total time required to complete a circle

        rospy.loginfo(f"Starting to move in a circle with radius {r} meters, will take {total_time:.2f} seconds to complete the circle.")
        
        start_time = rospy.Time.now()

        while not rospy.is_shutdown():
            elapsed_time = (rospy.Time.now() - start_time).to_sec()  # Calculate the time

            # If the time exceeds the total time, stop
            if elapsed_time >= total_time:
                twist.linear.x = 0.0
                twist.angular.z = 0.0
                self.cmd_vel_pub.publish(twist)  # stop the robot
                rospy.loginfo("Completed the circle and stopped.")
                break

            # Otherwise, continue to move along the circular trajectory
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

    def continue_move(self):
        twist = Twist()
        rate = rospy.Rate(10) #每秒执行10次，需要结合rate.sleep()使用
        
        keep_distance = 1.0 # 和墙壁保持0.1米的距离

        # while not rospy.is_shutdown(): 是 ROS 中用于确保节点持续运行的一个常见循环结构。这段代码用于在程序没有被终止的情况下，持续运行某些操作，直到 ROS 系统接收到停止指令（如按 Ctrl+C 终止节点，或其他系统关闭信号）为止。
        ## 这里判断的是d，也就是最近的距离
        while not rospy.is_shutdown():
            if self.min_distance < keep_distance - 0.1:
                twist.linear.x = 0
                twist.angular.z = 0.3
            elif self.min_distance > keep_distance + 0.1:
                twist.linear.x = 0.3
                twist.angular.z = 0.0
            
        self.cmd_vel_pub.publish(twist)
        rate.sleep() # 保持10hz的频率

        rospy.loginfo(f"Current minimum distance: {self.min_distance}")


if __name__ == '__main__':
    rospy.init_node('basic_mover')
    #BasicMover().out_and_back(1)
    #rospy.sleep(3)      # Let the robot wait for 3 seconds
    #BasicMover().draw_square(1)
    #rospy.sleep(3)
    #BasicMover().move_in_a_circle(1)
    # BasicMover().rotate_in_place()

    BasicMover().continue_move()

