#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-03-13 09:48:44
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-15 22:25:10
@Units        : None
@Description  : 让五架飞机都飞起来到指定的位置
@Dependencies : None
@NOTICE       : 
'''
#!/usr/bin/env python
# -*- coding: utf-8 -*-
import rospy
# from std_msgs.msg import String
from geometry_msgs.msg import Twist
import math
import time


def pub_vel_des_test():
    rospy.init_node('pub_all_uavs_fly')

    pubs = []
    pub_pose_uav1 = rospy.Publisher('/uav1/pose_cmd', Twist, queue_size=1)
    pubs.append(pub_pose_uav1)
    pub_pose_uav2 = rospy.Publisher('/uav2/pose_cmd', Twist, queue_size=1)
    pubs.append(pub_pose_uav2)
    pub_pose_uav3 = rospy.Publisher('/uav3/pose_cmd', Twist, queue_size=1)
    pubs.append(pub_pose_uav3)
    pub_pose_uav4 = rospy.Publisher('/uav4/pose_cmd', Twist, queue_size=1)
    pubs.append(pub_pose_uav4)
    # pub_pose_uav5 = rospy.Publisher('/uav5/pose_cmd', Twist, queue_size=1)
    # pubs.append(pub_pose_uav5)

    # pubs = [pub_pose_uav1, pub_pose_uav2,
    #         pub_pose_uav3, pub_pose_uav4]

    positions = [(0., 0.), (1., 1.), (2., 2.), (-1., -1.)]

    rate = rospy.Rate(10)
    i = 0
    start = time.time()
    while not rospy.is_shutdown():
        print(i)

        for index, (x, y) in enumerate(positions):
            cmd_input = Twist()
            cmd_input.linear.x = x
            cmd_input.linear.y = y
            cmd_input.linear.z = 1.5
            cmd_input.angular.x = 0.0
            cmd_input.angular.y = 0.0
            cmd_input.angular.z = 0.0
            pubs[index].publish(cmd_input)

        i = i+1
        rate.sleep()


if __name__ == '__main__':
    try:
        pub_vel_des_test()
    except rospy.ROSInterruptException:
        pass
