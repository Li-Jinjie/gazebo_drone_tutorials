#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-07 10:18:06
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-20 19:55:28
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
from geometry_msgs.msg import Twist
from agents.agent_COL_TGT import Agent_COL_TGT
from environments.gazebo_2_tasks_final import GazeboEnvironment2
from rl_glue import RLGlue
from tf.transformations import *


def callback(cmd_body):
    '''The callback function of this subscriber.
    data is based on the leader frame: uav1 frame
    '''
    # step1: 坐标变换
    cmd_world = Vector3()

    try:
        # is follower
        q = rl_glue.environment.leader_state.pose.orientation

        # 旋转后base_link坐标系在map坐标系中的姿态 × 目标点在base_link坐标系的中的位置 + base_link坐标系在map坐标系中的位置
        # r_matrix = trans.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        # t = np.dot(r_matrix, [cmd_body.x, cmd_body.y, 0]) + \
        #     [rl_glue.environment.leader_state.pose.position.x,
        #         rl_glue.environment.leader_state.pose.position.y, 0]

        # 参考四元数的变换： result = quaternion_multiply(quaternion_multiply(rot, vec), quaternion_conjugate(rot)) + [[t.x], [t.y], [t.z], [0]]

        rot = [q.x, q.y, q.z, q.w]
        vec = [cmd_body.x, cmd_body.y, 0, 0]
        p = rl_glue.environment.leader_state.pose.position
        result = quaternion_multiply(quaternion_multiply(
            rot, vec), quaternion_conjugate(rot)) + [p.x, p.y, p.z, 0]

        cmd_world.x = result[0]
        cmd_world.y = result[1]
    except:
        # is leader
        cmd_world.x = cmd_body.x
        cmd_world.y = cmd_body.y

    if abs(cmd_world.x - rl_glue.environment.target_x) < 0.02 and abs(cmd_world.y - rl_glue.environment.target_y) < 0.02:
        return

    x = cmd_world.x
    y = cmd_world.y

    # step2: 发指令

    start_time = time.clock()
    # Runs an episode while keeping track of visited states
    rl_glue.environment.target_x = x
    rl_glue.environment.target_y = y

    # 手动更新一下TGT任务的RL state
    env = rl_glue.environment
    rl_glue.agent.state_TGT = env.get_observation_TGT(
        env.current_state, env.target_position)

    state, action = rl_glue.rl_start()
    is_terminal = False
    print 'target:', rl_glue.environment.target_position
    while not is_terminal:
        # 手动更新一下TGT任务的RL state
        env = rl_glue.environment
        rl_glue.agent.state_TGT = env.get_observation_TGT(
            env.current_state, env.target_position)

        reward, state, action, is_terminal = rl_glue.rl_step()
    end_time = time.clock()
    print 'Total time:', end_time - start_time
    rl_glue.rl_cleanup()

    # step3: 到达目的地以后，校正朝向
    final_cmd = Twist()   # world frame

    final_cmd.linear.x = rl_glue.environment.current_state.pose.position.x
    final_cmd.linear.y = rl_glue.environment.current_state.pose.position.y
    final_cmd.linear.z = rl_glue.environment.current_state.pose.position.z

    try:
        # is follower
        q2 = rl_glue.environment.leader_state.pose.orientation
        rpy_leader = euler_from_quaternion([q2.x, q2.y, q2.z, q2.w])
        final_cmd.angular.z = rpy_leader[2]
    except:
        # is leader
        q1 = rl_glue.environment.current_state.pose.orientation
        rpy_current = euler_from_quaternion([q1.x, q1.y, q1.z, q1.w])
        final_cmd.angular.z = rpy_current[2]

    # step4: 到达位置后，调整朝向，且定在那里不动。
    start = time.clock()
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rl_glue.environment.pub_command.publish(final_cmd)
        # if time.clock() - start > 10:
        #     break
        rate.sleep()


if __name__ == "__main__":
    # initialization
    ns = rospy.get_namespace()
    name = re.findall(r'/(.*)/', ns)[0]
    # name = 'uav2'

    agent = Agent_COL_TGT
    env = GazeboEnvironment2

    agent_info = {
        'q_path': "/home/ljj/gazebo_drone_tutorials/catkin_ws/src/tabular_dyna_q/scripts/results/DynaQ_table_r1_e150.npy"}
    # env_info = {"end_radius": 0.05, "target_x": 1.2, "target_y": -1.2}
    env_info = {"end_radius": 0.08, 'robot_name': name,
                'target_x': 0.0, 'target_y': 0.0}

    rl_glue = RLGlue(env, agent)
    rl_glue.rl_init(agent_info, env_info)
    print "q_table_control node is running......."

    # rospy.init_node('q_table_controller', anonymous=False)
    # 在gazebo_env节点里已经包含了init了节点
    rospy.Subscriber('rl_cmd', Vector3, callback)
    rospy.spin()
