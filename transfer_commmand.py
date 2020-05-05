#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-04 18:48:56
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-05 10:41:46
@Units        : None
@Description  : 写一个函数，实现一个功能：收到一个基于机体坐标系的指令，发出一个基于world坐标系的指令。
@Dependencies : None
@NOTICE       : None
'''

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
import tf.transformations as trans
import numpy as np
import angles

PI = 3.1415926


class Test():
    def __init__(self):
        self.current_pose = Twist()  # 数据类型是Twist
        self.current_twist = Twist()
        self.command = Twist()  # based on the base_link frame

        self.sub_state = rospy.Subscriber(
            '/uav1/ground_truth/state', Odometry, self.state_callback)

        self.pub_command = rospy.Publisher(
            '/uav1/pose_cmd', Twist, queue_size=10)
        rospy.init_node('imu_processing', anonymous=True)

    def state_callback(self, msg):
        self.current_twist = msg.twist.twist
        self.current_pose.linear = msg.pose.pose.position
        rpy = trans.euler_from_quaternion(
            [msg.pose.pose.orientation.x,
                msg.pose.pose.orientation.y,
                msg.pose.pose.orientation.z,
                msg.pose.pose.orientation.w])  # rpy是弧度
        # current_pose.angular.x = rpy[0]
        # current_pose.angular.y = rpy[1]
        self.current_pose.angular.z = rpy[2]
        # print('current_pose', self.current_pose)
        # print('current_twist', self.current_twist)

    def trans_command(self, command):
        ''' Transfers the command from base_link to world frame.
        Input: pose command under base_link frame. Twist()
        Output: new pose command under world frame. Twist()
        '''
        current_pose = self.current_pose

        trans_cmd = Twist()

        # step 1: angle of z axis
        trans_cmd.angular.z = current_pose.angular.z + command.angular.z
        # if out of range []
        if trans_cmd.angular.z > PI:
            trans_cmd.angular.z -= 2 * PI
        elif trans_cmd.angular.z < - PI:
            trans_cmd.angular.z += 2 * PI

        # step 2: position
        if command.linear.x != 0:
            # 旋转后base_link坐标系在map坐标系中的姿态 × 目标点在base_link坐标系的中的位置 + base_link坐标系在map坐标系中的位置
            r_matrix = trans.euler_matrix(
                trans_cmd.angular.x,
                trans_cmd.angular.y,
                trans_cmd.angular.z
            )
            t = np.dot(r_matrix[:3, :3], [command.linear.x, command.linear.y, command.linear.z]) + \
                [current_pose.linear.x, current_pose.linear.y, current_pose.linear.z]
            trans_cmd.linear.x = t[0]
            trans_cmd.linear.y = t[1]
            trans_cmd.linear.z = current_pose.linear.z
        else:
            trans_cmd.linear = current_pose.linear

        return trans_cmd

    def publish_cmd(self, command):
        '''Receives a command under base_link frame, sends it to pose_cmd node and waits until the uav stops.
        Input: pose_command, Twist()
        Output: None
        '''
        # Transfers the command between two frames.
        world_cmd = self.trans_command(command)

        # publishes the command until the uav reaches the target
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            # 0.5 degree, 1 cm
            if (abs(world_cmd.angular.z - self.current_pose.angular.z) < 0.008) \
                and (abs(world_cmd.linear.x - self.current_pose.linear.x) < 0.01) \
                    and (abs(world_cmd.linear.y - self.current_pose.linear.y) < 0.01):
                break
            self.pub_command.publish(world_cmd)
            print 'current_pose', self.current_pose
            print 'target_pose', world_cmd
            rate.sleep()


if __name__ == '__main__':
    test_obj = Test()

    command = Twist()
    command.angular.z = -90 * PI / 180  # from radians to degrees
    command.linear.x = -0.6

    try:
        test_obj.publish_cmd(command)
        # rate = rospy.Rate(1)  # 10hz
        # while not rospy.is_shutdown():
        #     command_new = test_obj.trans_command(command)
        #     test_obj.pub_command.publish(command_new)
        #     rospy.sleep(5)
    except rospy.ROSInterruptException:
        pass
