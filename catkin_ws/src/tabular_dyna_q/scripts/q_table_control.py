#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-07 10:18:06
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-16 11:10:32
@Units        : None
@Description  : The usage of q-table to control the uav flying to the target
@Dependencies : None
@NOTICE       : None
'''
import os
import sys
import time
import re
import numpy as np
import rospy
from geometry_msgs.msg import Vector3
from agents.agent_completed import AgentCompleted
from environments.gazebo_env_completed import GazeboEnvironmentCommpleted
from rl_glue import RLGlue


def callback(data):
    '''The callback function of this subscriber.
    data is based on the leader frame: uav1 frame
    '''
    # 坐标变换

    if (data.x == rl_glue.environment.target_x) and (data.y == rl_glue.environment.target_y):
        return

    x = data.x
    y = data.y

    start_time = time.clock()
    # Runs an episode while keeping track of visited states
    rl_glue.environment.target_x = x
    rl_glue.environment.target_y = y
    state, action = rl_glue.rl_start()
    is_terminal = False
    while not is_terminal:
        reward, state, action, is_terminal = rl_glue.rl_step()
    end_time = time.clock()
    print 'Total time:', end_time - start_time


if __name__ == "__main__":
    # initialization
    ns = rospy.get_namespace()
    name = re.findall(r'/(.*)/', ns)[0]
    # name = 'uav1'

    agent = AgentCompleted
    env = GazeboEnvironmentCommpleted

    agent_info = {
        'q_path': "/home/ljj/gazebo_drone_tutorials/catkin_ws/src/tabular_dyna_q/scripts/results/DynaQ_table_r1_e150.npy"}
    # env_info = {"end_radius": 0.05, "target_x": 1.2, "target_y": -1.2}
    env_info = {"end_radius": 0.05, 'robot_name': name,
                'target_x': 0.0, 'target_y': 0.0}

    rl_glue = RLGlue(env, agent)
    rl_glue.rl_init(agent_info, env_info)
    print "q_table_control node is running......."

    # rospy.init_node('q_table_controller', anonymous=False)
    # 在gazebo_env节点里已经包含了init了节点
    rospy.Subscriber('rl_cmd', Vector3, callback)
    rospy.spin()

    # while True:
    #     flag = raw_input("Do you want to quit? (T/F)")
    #     if flag == 'T':
    #         print "Quit successfully!"
    #         break
    #     else:
    #         pass

    #     start_time = time.clock()

    #     x = raw_input("Please input the x coordination:")
    #     x = float(x)
    #     y = raw_input("Please input the y coordination:")
    #     y = float(y)

    #     # Runs an episode while keeping track of visited states
    #     rl_glue.environment.target_x = x
    #     rl_glue.environment.target_y = y
    #     state, action = rl_glue.rl_start()
    #     is_terminal = False
    #     while not is_terminal:
    #         reward, state, action, is_terminal = rl_glue.rl_step()

    #     end_time = time.clock()

    #     print "The time of episode:", end_time - start_time
