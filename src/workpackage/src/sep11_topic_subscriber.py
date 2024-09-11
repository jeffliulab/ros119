#!/usr/bin/env python

import rospy
from std_msgs.msg import Int32
# 从std_msgs包中导入了Int32消息类型。
# Int32是ROS中的一种标准消息类型，表示32位整数。
# std_msgs 是一个标准消息包（standard message package），是ROS（机器人操作系统）中的一部分，用来定义一些基础的数据类型。
# 这些基础数据类型常用于ROS系统中不同节点之间的消息传递。
# 这些消息类型可以用于ROS话题（topics）、服务（services）和操作（actions）之间的数据传递，帮助开发者方便地处理和传输基础的数据类型。
# 因此，std_msgs 可以看作是ROS中的基础消息定义库，提供了许多通用的消息类型，便于开发者构建和通信机器人应用。

def callback(msg):
    print (msg.data)     # 这行会在控制台打印收到消息中的数据。
    # 这个函数定义了一个回调函数，当订阅者收到 counter 话题的消息时，这个函数会被触发，并打印消息内容 msg.data。
    # 'msg' 参数表示订阅者接收到的消息。

# 回调函数（Callback function）是编程中的一个概念，指的是一个函数被作为参数传递给另一个函数，并在适当的时候由后者调用。
# 回调函数用于处理异步任务，或者当某些特定事件发生时执行指定的代码逻辑。
# 在ROS（机器人操作系统）中，回调函数主要用于处理节点订阅到的消息。
# 当一个ROS节点通过订阅某个话题接收到消息时，ROS会调用开发者定义的回调函数来处理这个消息。换句话说，当消息到达时，回调函数会自动被触发并执行。
# 回调函数的作用在于它可以让程序等待特定的事件发生时自动做出反应，而不需要程序主动轮询或一直检查某个条件，这样能提高代码的效率和简洁性。


rospy.init_node('topic_subscriber')
# 这行初始化了一个名为'topic_subscriber'的新ROS节点。
# 每个ROS节点必须有唯一的名称，这确保了节点已准备好参与ROS通信。

sub = rospy.Subscriber('counter', Int32, callback)
# 这行创建了一个订阅者，订阅名为'counter'的话题，该话题的消息类型为Int32。
# 每当在'counter'话题上发布消息时，'callback'函数就会执行。
# 在这行代码中，'counter' 是话题名称，Int32 是消息类型，而 callback 是回调函数。

rospy.spin()
# 这个函数让节点持续运行，防止它退出。
# 它等待回调函数被触发，即当订阅的话题上有消息到达时调用回调函数。