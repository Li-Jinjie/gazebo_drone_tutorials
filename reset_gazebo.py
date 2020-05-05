#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-05 10:42:33
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-05 11:15:45
@Units        : None
@Description  : file content
@Dependencies : None
@NOTICE       : None
'''
import rospy
import copy
import numpy as np
from gazebo_msgs.msg import ModelState


class Env():
    def __init__(self):
        rospy.init_node('Reset_World', anonymous=False)
        # -----------Default Robot State-----------------------
        self.set_self_state = ModelState()
        self.set_self_state.model_name = 'uav1'
        self.set_self_state.pose.position.x = 0.0
        self.set_self_state.pose.position.y = 0.0
        self.set_self_state.pose.position.z = 1.5
        self.set_self_state.pose.orientation.x = 0.0
        self.set_self_state.pose.orientation.y = 0.0
        self.set_self_state.pose.orientation.z = 0.0
        self.set_self_state.pose.orientation.w = 1.0
        self.set_self_state.twist.linear.x = 0.
        self.set_self_state.twist.linear.y = 0.
        self.set_self_state.twist.linear.z = 0.
        self.set_self_state.twist.angular.x = 0.
        self.set_self_state.twist.angular.y = 0.
        self.set_self_state.twist.angular.z = 0.
        self.set_self_state.reference_frame = 'world'

        self.set_state = rospy.Publisher(
            '/gazebo/set_model_state', ModelState, queue_size=10)

    def SetObjectPose(self, name='uav1', random_flag=False):
        # if name is 'uav1':
        #     # rand = random.random()
        object_state = copy.deepcopy(self.set_self_state)
        print 'object_state=', object_state

        # quaternion = tf.transformations.quaternion_from_euler(0., 0., np.random.uniform(-np.pi, np.pi))

        # object_state.pose.orientation.x = quaternion[0]
        # object_state.pose.orientation.y = quaternion[1]
        # object_state.pose.orientation.z = quaternion[2]
        # object_state.pose.orientation.w = quaternion[3]
        # else:
        #     object_state = self.States2State(self.default_states, name)
        for i in range(5):
            self.set_state.publish(object_state)
        print 'reset model successfully!!!!'

    def ResetWorld(self):
        self.total_evaluation = 0
        self.SetObjectPose()  # reset robot
        # for x in xrange(len(self.object_name)):
        # 	self.SetObjectPose(self.object_name[x]) # reset target
        self.self_speed = [.4, 0.0]
        self.step_target = [0., 0.]
        self.step_r_cnt = 0.
        self.start_time = time.time()
        rospy.sleep(0.5)


if __name__ == "__main__":
    env = Env()
    env.SetObjectPose()
