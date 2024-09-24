#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
from std_msgs.msg import Int32  # 导入 Int32 类型

def scan_callback(msg):
    global closest_obstacle_dist   # 全局变量：最近障碍物的距离
    infinite_distance = 10         # 将10m视为假象中无限远的距离，这样是为了避免传输非数字距离信息

    num_ranges = len(msg.ranges)    # 获取激光雷达的数据点数量

    ### 调试时用的
    print("The amount of data points of laser is: ", num_ranges) # Print the information
    ### 这里证明laser一共有360个点位

    ### 在本课程的机器人安装中
    ### 0 是正前方（foreward）
    foreward_distance = msg.ranges[0] if msg.ranges[0] != float('inf') else infinite_distance
    print("\nforeward distance: ", foreward_distance)
    ### 90 是正左方
    left_distance = msg.ranges[90] if msg.ranges[90] != float('inf') else infinite_distance
    print("\nleft distance: ", left_distance)
    ### 180 是正后方
    backward_distance = msg.ranges[180] if msg.ranges[180] != float('inf') else infinite_distance
    print("\nbackward distance: ", backward_distance)
    ### 270 是正右方
    right_distance = msg.ranges[270] if msg.ranges[270] != float('inf') else infinite_distance
    print("\nright distance: ", right_distance)

    ### Typically 0 degrees is foreward. and then we go counter clockwise around positive up to 359

    # 计算最近的障碍物距离
    # msg.ranges 是一个包含激光雷达所有扫描方向距离数据的数组，数组的每个元素表示从机器人到障碍物的距离。
    # closest_obstacle_dist = min(msg.ranges)
    # 改进：忽略那些无限大的点
    valid_closest_dist = min([r for r in msg.ranges if r != float('inf')])
    closest_obstacle_dist = valid_closest_dist if valid_closest_dist != float('inf') else infinite_distance
    rospy.loginfo(f"Closest obstacle distance: {closest_obstacle_dist}") # 打印调试信息

    closest_direction = msg.ranges.index(closest_obstacle_dist)
    rospy.loginfo(f"Closest distance direction: {closest_direction}") # 打印调试信息


    # 发布信息
    min_distance_pub.publish(closest_obstacle_dist) # 发布最近距离的距离值
    min_distance_direction_pub.publish(closest_direction) # 发布最近距离的方向值
    foreward_distance_pub.publish(foreward_distance) # 发布正前方的距离
    left_distance_pub.publish(left_distance) # 发布正左方的距离
    backward_distance_pub.publish(backward_distance) # 发布正后方的距离
    right_distance_pub.publish(right_distance) # 发布正右方的距离
    
# 主程序
if __name__ == '__main__':
    rospy.init_node('laser_scan_listener')

    # 订阅激光雷达数据
    # LaserScan 是机器人传感器（通常是激光雷达）自动发布的数据类型，前提是机器人本身装备了激光雷达（LiDAR）传感器，并且已经配置好发布激光雷达的扫描数据。
    # 在ROS环境下，当你使用一个带有激光雷达的机器人时，它会自动发布激光雷达的扫描数据到某个话题（例如 /scan）。这些数据通常由传感器驱动程序（比如 hokuyo_node 或 urg_node，视具体的激光雷达品牌和型号而定）自动发布。
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)

    # 发布各个变量到对应的topic （这里是为了方便，如果要放到项目上，这里需要设置一个topic的message类型，包含这些变量）
    min_distance_pub = rospy.Publisher('/min_distance_pub', Float32, queue_size=1)
    min_distance_direction_pub = rospy.Publisher('/min_distance_direction_pub', Int32, queue_size=1)
    foreward_distance_pub = rospy.Publisher('/foreward_distance_pub', Float32, queue_size=1)
    left_distance_pub = rospy.Publisher('/left_distance_pub', Float32, queue_size=1)
    backward_distance_pub = rospy.Publisher('/backward_distance_pub', Float32, queue_size=1)
    right_distance_pub = rospy.Publisher('/right_distance_pub', Float32, queue_size=1)

    rospy.spin()  # 保持节点运行