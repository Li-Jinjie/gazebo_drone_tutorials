#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-04 18:48:56
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-05 16:36:22
@Units        : None
@Description  : 写一个函数，实现一个功能：收到一个基于机体坐标系的指令，发出一个基于world坐标系的指令。
@Dependencies : None
@NOTICE       : 实例和变量用驼峰命名法，函数和类用下划线命名法
'''

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
import copy
import tf.transformations as trans
import numpy as np
import angles

PI = 3.1415926


class Test():
    def __init__(self):
        rospy.init_node('imu_processing', anonymous=True)

        # -----------Default Robot State-----------------------
        self.initState = ModelState()
        self.initState.model_name = 'uav1'
        self.initState.pose.position.x = 0.0
        self.initState.pose.position.y = 0.0
        self.initState.pose.position.z = 1.5
        self.initState.pose.orientation.x = 0.0
        self.initState.pose.orientation.y = 0.0
        self.initState.pose.orientation.z = 0.0
        self.initState.pose.orientation.w = 1.0
        self.initState.twist.linear.x = 0.
        self.initState.twist.linear.y = 0.
        self.initState.twist.linear.z = 0.
        self.initState.twist.angular.x = 0.
        self.initState.twist.angular.y = 0.
        self.initState.twist.angular.z = 0.
        self.initState.reference_frame = 'world'

        # ------------------parameters---------------

        self.modelStates = ModelStates()  # multiple models, quaternion
        self.command = Twist()  # based on the base_link frame, rpy

        # -----------Publisher and Subscriber-------------
        self.sub_state = rospy.Subscriber(
            '/gazebo/model_states', ModelStates, self.state_callback)

        self.pub_command = rospy.Publisher(
            '/uav1/pose_cmd', Twist, queue_size=10)

        self.set_state = rospy.Publisher(
            '/gazebo/set_model_state', ModelState, queue_size=10)

    def set_object_pose(self, name='uav1', random_flag=False):
        object_state = copy.deepcopy(self.initState)

        rate = rospy.Rate(10)  # 10hz
        while not rospy.is_shutdown():
            if self.currentPose == self.initState.pose:
                break
            self.set_state.publish(object_state)
            rate.sleep()
        print 'Successfully set the target!'

    def state_callback(self, msg):
        '''
        update model states
        '''
        if msg.model_name == 'uav1':
            self.currentTwist = msg.twist
            self.currentPose.linear = msg.pose.position
            rpy = trans.euler_from_quaternion(
                [msg.pose.orientation.x,
                    msg.pose.orientation.y,
                    msg.pose.orientation.z,
                    msg.pose.orientation.w])  # rpy是弧度
            currentPose.angular.x = 0
            currentPose.angular.y = 0
            self.currentPose.angular.z = rpy[2]
            print('currentPose', self.currentPose)
            print('currentTwist', self.currentTwist)

    def trans_command(self, command, name='uav1'):
        ''' Transfers the command from base_link to world frame.
        Input: pose command under base_link frame. Twist()
        Output: new pose command under world frame. Twist()
        '''
        currentPose = Twist()
        currentPose.linear = self.modelStates.
        print 'currentPose', currentPose

        transCmd = Twist()

        # step 1: angle of z axis
        transCmd.angular.z = currentPose.angular.z + command.angular.z
        # if out of range []
        if transCmd.angular.z > PI:
            transCmd.angular.z -= 2 * PI
        elif transCmd.angular.z < - PI:
            transCmd.angular.z += 2 * PI

        # step 2: position
        if command.linear.x != 0:
            # 旋转后base_link坐标系在map坐标系中的姿态 × 目标点在base_link坐标系的中的位置 + base_link坐标系在map坐标系中的位置
            r_matrix = trans.euler_matrix(
                transCmd.angular.x,
                transCmd.angular.y,
                transCmd.angular.z
            )
            t = np.dot(r_matrix[:3, :3], [command.linear.x, command.linear.y, command.linear.z]) + \
                [currentPose.linear.x, currentPose.linear.y, currentPose.linear.z]
            transCmd.linear.x = t[0]
            transCmd.linear.y = t[1]
            transCmd.linear.z = currentPose.linear.z
        else:
            transCmd.linear = currentPose.linear

        return transCmd

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
            if (abs(world_cmd.angular.z - self.currentPose.angular.z) < 0.008) \
                and (abs(world_cmd.linear.x - self.currentPose.linear.x) < 0.01) \
                    and (abs(world_cmd.linear.y - self.currentPose.linear.y) < 0.01):
                break
            self.pub_command.publish(world_cmd)
            # print 'currentPose', self.currentPose
            # print 'target_pose', world_cmd
            rate.sleep()


if __name__ == '__main__':
    testObj = Test()

    command = Twist()
    command.angular.z = -90 * PI / 180  # from radians to degrees
    command.linear.x = -0.6

    try:
        testObj.publish_cmd(command)
        # rate = rospy.Rate(1)  # 10hz
        # while not rospy.is_shutdown():
        #     command_new = testObj.trans_command(command)
        #     testObj.pub_command.publish(command_new)
        #     rate.sleep()
    except rospy.ROSInterruptException:
        pass
