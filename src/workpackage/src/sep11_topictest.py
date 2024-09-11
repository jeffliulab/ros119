#!/usr/bin/env python
# 第一是 shebang，用于告诉系统这个脚本应该使用哪个解释器来执行。在这个例子中，它指向了 Python 解释器。

import rospy
# import rospy：导入 rospy 库，这是 Python 中与 ROS 交互的库，提供了节点的创建、消息发布、订阅等功能。

from std_msgs.msg import Int32
# from std_msgs.msg import Int32：从标准消息库中导入 Int32 数据类型，这个类型代表32位整数。稍后我们将使用这个类型来发布整数消息。

rospy.init_node('topic_publisher')
# rospy.init_node()：初始化一个 ROS 节点，这个节点被命名为 'topic_publisher'。
# 在 ROS 中，每个独立的执行单元（如程序）被称为“节点”。
# 节点需要通过 init_node() 来启动，这样才能与 ROS 主控（roscore）进行通信。

pub = rospy.Publisher('counter', Int32)
# rospy.Publisher()：创建一个发布者对象 pub，它将发布名为 counter 的 topic，消息的类型为 Int32（即32位整数）。
# 其他 ROS 节点可以订阅这个 topic 并接收由该节点发布的消息。

rate = rospy.Rate(0.1)
# rospy.Rate()：定义发布消息的频率。在这个例子中，Rate(2) 表示每秒发布 2 次消息，即每隔 0.5 秒发布一次。

count = 0
while not rospy.is_shutdown(): # 这是一个循环，持续运行直到节点关闭。
    pub.publish(count)
    count += 1
    rate.sleep() # 这个函数确保循环按设定的频率执行，在这里，它会让节点每 0.5 秒（2Hz）执行一次循环，保证消息的发布频率。
