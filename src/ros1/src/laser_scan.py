#!/usr/bin/env python3

import rospy
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32


closest_obstacle_dist = 0 # 全局变量：最近障碍物的距离

def scan_callback(msg):
    global closest_obstacle_dist

    # 获取激光雷达的数据点数量
    num_ranges = len(msg.ranges)

    # 计算最近的障碍物距离
    # msg.ranges 是一个包含激光雷达所有扫描方向距离数据的数组，数组的每个元素表示从机器人到障碍物的距离。
    closest_obstacle_dist = min(msg.ranges)

    # 计算正前方障碍物的距离
    front_index = int((0 - msg.angle_min) / msg.angle_increment)
    front_obstacle_dist = msg.ranges[front_index]



    # 打印调试信息
    rospy.loginfo(f"Closest obstacle distance: {closest_obstacle_dist}")

    # 发布信息（这里选择发布正前方的障碍物的距离）
    obstacle_distance_pub.publish(front_obstacle_dist)

# 主程序
if __name__ == '__main__':
    rospy.init_node('laser_scan_listener')

    # 订阅激光雷达数据
    # LaserScan 是机器人传感器（通常是激光雷达）自动发布的数据类型，前提是机器人本身装备了激光雷达（LiDAR）传感器，并且已经配置好发布激光雷达的扫描数据。
    # 在ROS环境下，当你使用一个带有激光雷达的机器人时，它会自动发布激光雷达的扫描数据到某个话题（例如 /scan）。这些数据通常由传感器驱动程序（比如 hokuyo_node 或 urg_node，视具体的激光雷达品牌和型号而定）自动发布。
    scan_sub = rospy.Subscriber('/scan', LaserScan, scan_callback)

    # 发布最近障碍物距离到 /obstacle_distance
    obstacle_distance_pub = rospy.Publisher('/obstacle_distance', Float32, queue_size=1)


    

    rospy.spin()  # 保持节点运行














