#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-04 18:48:56
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-20 19:55:07
@Units        : Meter, radian (if no description)
@Description  : env类，与gazebo联动,可生成避障(collision avoidance, COL)与目标追踪(target seek, TGT)两个任务对应的状态. To implement the collision avoidance task.
@Dependencies : None
@NOTICE       : current_state 存的是机器人在world中的状态，target_position存的是要飞到的位置，RL中的state其实是target_position相对current_state的位置，在get_observation_TGT()中可以看到具体使用。
'''

from template_environment import BaseEnvironment

import rospy
from scipy import io
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
import copy
import tf.transformations as trans
import numpy as np
import math


class GazeboEnvironment2(BaseEnvironment):
    """Implements the environment for an RLGlue environment

    Note:
        env_init, env_start, env_step, env_cleanup, and env_message are required
        methods.
    """

    def env_init(self, env_info):
        """Setup for the environment called when the experiment first starts.

        Note:
            Initialize a tuple with the reward, first state observation, boolean
            indicating if it's terminal.
        """
        # initialize
        try:
            rospy.init_node('Gazebo_Env', anonymous=False)
        except rospy.ROSInitException:
            print "Error: 初始化节点失败，检查gazebo环境是否提前运行。"

        self.trajectory = []

        # -----------Default Robot State-----------------------
        self.randomFlag = env_info.get('random_flag', False)
        self.robot_name = env_info.get('robot_name', 'uav1')
        self.target_x = env_info.get('target_x', 2.0)
        self.target_y = env_info.get('target_y', 2.0)
        # 默认设置的范围内没障碍物
        self.obstacle_x = env_info.get('obstacle_x', 100.0)
        self.obstacle_y = env_info.get('obstacle_y', 100.0)
        self.fuck_rotate = 0

        self.leader_name = 'uav1'

        self.init_robot_state = ModelState()
        self.init_robot_state.model_name = self.robot_name
        self.init_robot_state.pose.position.x = 0.0
        self.init_robot_state.pose.position.y = 0.0
        self.init_robot_state.pose.position.z = 1.5
        self.init_robot_state.pose.orientation.x = 0.0
        self.init_robot_state.pose.orientation.y = 0.0
        self.init_robot_state.pose.orientation.z = 0.0
        self.init_robot_state.pose.orientation.w = 1.0
        self.init_robot_state.twist.linear.x = 0.
        self.init_robot_state.twist.linear.y = 0.
        self.init_robot_state.twist.linear.z = 0.
        self.init_robot_state.twist.angular.x = 0.
        self.init_robot_state.twist.angular.y = 0.
        self.init_robot_state.twist.angular.z = 0.
        self.init_robot_state.reference_frame = 'world'

        # ----------- Parameters in RL env class -----------
        self.maze_dim_TGT = {'num_fan': 11,
                             'num_ring': 11}   # num_fan, num_ring
        self.maze_dim_COL = {'num_fan': 9, 'num_ring': 5}

        origin = self.init_robot_state.pose.position
        # coordination of the target point.
        self.target_position = (
            origin.x + self.target_x, origin.y + self.target_y, origin.z)

        # coordination of the obstacle point.
        self.obstacle_position = (self.obstacle_x, self.obstacle_y, origin.z)

        # self.end_state = [0, 0]
        self.end_radius = env_info["end_radius"]  # meters
        # The robot's pose and twist infomation under world frame
        self.current_state = ModelState()

        # The leader's pose and twist information under world frame
        if self.leader_name != self.robot_name:
            self.leader_state = ModelState()

        # -----------Publisher and Subscriber-------------
        self.sub_state = rospy.Subscriber(
            '/gazebo/model_states', ModelStates, self.state_callback)

        self.set_state = rospy.Publisher(
            '/gazebo/set_model_state', ModelState, queue_size=10)

        self.pub_command = rospy.Publisher(
            rospy.get_namespace()+'pose_cmd', Twist, queue_size=10)
        # self.pub_command = rospy.Publisher(
        # '/uav2/pose_cmd', Twist, queue_size = 10)

        rospy.sleep(2.)
        # # What function to call when you ctrl + c
        # rospy.on_shutdown(self.shutdown)

        self.reward_obs_term = {
            'TGT': [0.0, None, False], 'COL': [0.0, None, False]}

    def state_callback(self, States):
        '''The callback function of Subscriber: sub_state.
        Update the current state of robot 'robot_name'
        '''
        self_index = States.name.index(self.robot_name)
        self.current_state.model_name = self.robot_name
        self.current_state.pose = States.pose[self_index]
        self.current_state.twist = States.twist[self_index]
        self.current_state.reference_frame = 'world'

        # if not leader
        if self.leader_name != self.robot_name:
            leader_index = States.name.index(self.leader_name)
            self.leader_state.model_name = self.leader_name
            self.leader_state.pose = States.pose[leader_index]
            self.leader_state.twist = States.twist[leader_index]
            self.leader_state.reference_frame = 'world'

        min_distance = None
        self_x = self.current_state.pose.position.x
        self_y = self.current_state.pose.position.y

        for i in range(len(States.name)):
            # 不是本机
            if i != self_index:
                x = States.pose[i].position.x
                y = States.pose[i].position.y

                relative_dis = np.sqrt((x - self_x)**2 + (y - self_y)**2)
                if relative_dis < min_distance or min_distance == None:
                    min_distance = relative_dis
                    min_x = x
                    min_y = y

        if min_distance != None:
            self.obstacle_x = (0.5/min_distance) * self_x + \
                (min_distance - 0.5)/min_distance * min_x
            self.obstacle_y = (0.5/min_distance) * self_y + \
                (min_distance - 0.5)/min_distance * min_y
            # print 'obstacle: ', self.obstacle_x, self.obstacle_y

    def set_robot_state(self, state):
        '''Set the model state equals to state
        In: target ModelState()
        Out: None.
        '''
        rate = rospy.Rate(10)
        while True:
            current_pose = self.current_state.pose
            self.set_state.publish(state)
            # Make sure the command above is accepted correctly.
            if (abs(0 - state.pose.orientation.z) < 0.008) \
                and (abs(current_pose.position.x - state.pose.position.x) < 0.01) \
                    and (abs(current_pose.position.y - state.pose.position.y) < 0.01):
                break
            rate.sleep()

    def send_cmd_until_stop(self, command, transfer_flag=False):
        '''Receives a command under base_link frame, sends it to pose_cmd node and waits until the uav stops.
        Input: pose_command, Twist()
        Output: None
        '''
        # Transfers the command between two frames.
        if transfer_flag == True:
            world_cmd = self.coor_trans(command, self.current_state)
        else:
            world_cmd = command

        # publishes the command until the uav reaches the target
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            pose = self.current_state.pose
            q = pose.orientation
            rpy = trans.euler_from_quaternion([q.x, q.y, q.z, q.w])  # radians

            # 0.5 degree, 1 cm
            if (abs(world_cmd.angular.z - rpy[2]) < 0.008) \
                and (abs(world_cmd.linear.x - pose.position.x) < 0.01) \
                    and (abs(world_cmd.linear.y - pose.position.y) < 0.01):
                # print 'Arrive at the destination!'
                break
            self.pub_command.publish(world_cmd)
            # print 'currentPose', self.currentPose
            # print 'target_pose', world_cmd
            rate.sleep()

    def env_start(self):
        """The first method called when the experiment starts, called before the
        agent starts.

        Returns:
            The first state observation from the environment.
        """
        # # set the random target position
        # if self.randomFlag == True:
        #     self.target_x = np.random.rand() * 3.6 - 1.8
        #     self.target_x = round(self.target_x, 1)
        #     self.target_y = np.random.rand() * 3.6 - 1.8
        #     self.target_y = round(self.target_y, 1)

        #     origin = self.init_robot_state.pose.position
        #     self.target_position = (
        #         origin.x + self.target_x, origin.y + self.target_y, origin.z)
        #     print 'target_position = ', self.target_position

        # # reset the current state = init_state
        # self.set_robot_state(self.init_robot_state)

        self.target_position = (
            self.target_x, self.target_y, self.init_robot_state.pose.position.z)
        self.obstacle_position = (
            self.obstacle_x, self.obstacle_y, self.init_robot_state.pose.position.z)

        self.trajectory.append(
            (self.current_state.pose.position.x, self.current_state.pose.position.y))

        # get_observation
        self.reward_obs_term['TGT'][1] = self.get_observation_TGT(
            self.current_state, self.target_position)

        self.reward_obs_term['COL'][1] = self.get_observation_COL(
            self.current_state, self.obstacle_position)

        # return RL state
        return self.reward_obs_term['COL'][1]

    # check if current state is within the gridworld and return bool
    def out_of_bounds(self, cmd_transferred, current_state, target_position, rl_state):
        '''Determine if the action could be executed
        # Input: cmd_transferred: Twist(), current_state: ModelState(),
        target_position: tuple(3), rl_state: int
        # Output: Boolean
        '''
        if rl_state >= 110:  # in the outermost circle
            dis_old = np.sqrt((current_state.pose.position.x -
                               target_position[0])**2 + (current_state.pose.position.y - target_position[1])**2)
            dis_new = np.sqrt((cmd_transferred.linear.x -
                               target_position[0])**2 + (cmd_transferred.linear.y - target_position[1])**2)
            # if the new action leads to a further result
            if dis_new > dis_old:
                return True
            else:
                return False
        else:
            return False

    def get_observation_TGT(self, robot_state, target_position):
        '''
        give coordinations in continuous space (state), return state_observation.
        In: robot_state, ModelState(); target_position, tuple(3)
        Returns: state_observation, int
        '''
        # in the world frame
        x_relative = target_position[0] - robot_state.pose.position.x
        y_relative = target_position[1] - robot_state.pose.position.y

        # coordinates transfer
        # 在base系中的坐标 = world系在base系中的旋转矩阵(base系在world系中的旋转矩阵求逆) × 在world系中的坐标
        # 坐标变换一定要万分小心仔细
        q = robot_state.pose.orientation
        r_matrix = trans.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        r_matrix_inverse = trans.inverse_matrix(r_matrix)
        relative_coor_base = np.dot(
            r_matrix_inverse, [x_relative, y_relative, 0])

        # print 'relative_coor_base =', relative_coor_base

        # get RL state
        state = self.get_polar_coor_TGT(
            relative_coor_base[0], relative_coor_base[1])
        return state[0] * self.maze_dim_TGT['num_fan'] + state[1]

    def get_polar_coor_TGT(self, x, y):
        '''get the polar coordinate of Rectangular coordinate (x,y) in the target seek mission
        '''
        state = [0, 0]  # ring, fan
        dis_borders = [(0.0, 0.05), (0.05, 0.1), (0.1, 0.2),
                       (0.2, 0.3), (0.3, 0.4), (0.4, 0.5),
                       (0.5, 0.7), (0.7, 0.9), (0.9, 1.1),
                       (1.1, 1.3), (1.3, 1.5)]
        degree_borders = [(-5, +5), (5, 15), (15, 45), (45, 90),
                          (90, 135), (135, 180), (-180, -135), (-135, -90), (-90, -45), (-45, -15), (-15, -5)]

        distance = np.sqrt(x ** 2 + y ** 2)
        if distance > 1.5:
            state[0] = 10
        else:
            for i, (down, up) in enumerate(dis_borders):
                if distance >= down and distance < up:
                    state[0] = i
                    break

        degree = math.atan2(y, x) * 180 / math.pi
        # print 'degree = ', degree

        # (-180, 180]
        if degree <= -180:
            degree += 360
        elif degree > 180:
            degree -= 360
        # print 'degree2 =', degree

        for i, (down, up) in enumerate(degree_borders):
            if degree > down and degree <= up:
                state[1] = i
                break

        return state

    def get_observation_COL(self, robot_state, obstacle_position):
        '''
        give coordinations in continuous space (state), return state_observation.
        In: robot_state, ModelState(); obstacle_position, tuple(3)
        Returns: state_observation, int
        '''
        # in the world frame
        x_relative = obstacle_position[0] - robot_state.pose.position.x
        y_relative = obstacle_position[1] - robot_state.pose.position.y

        # coordinates transfer
        # 在base系中的坐标 = world系在base系中的旋转矩阵(base系在world系中的旋转矩阵求逆) × 在world系中的坐标
        # 坐标变换一定要万分小心仔细
        q = robot_state.pose.orientation
        r_matrix = trans.quaternion_matrix([q.x, q.y, q.z, q.w])[: 3, : 3]
        r_matrix_inverse = trans.inverse_matrix(r_matrix)
        relative_coor_base = np.dot(
            r_matrix_inverse, [x_relative, y_relative, 0])

        # print 'relative_coor_base =', relative_coor_base

        # get RL state
        state = self.get_polar_coor_COL(
            relative_coor_base[0], relative_coor_base[1])
        return state[0] * self.maze_dim_COL['num_fan'] + state[1]

    def get_polar_coor_COL(self, x, y):
        '''get the polar coordinate of Rectangular coordinate (x,y) in COL mission
        '''
        state = [0, 0]  # ring, fan
        dis_borders = [(0.0, 0.5), (0.5, 0.6), (0.6, 0.7),
                       (0.7, 0.8), (0.8, 0.9)]
        degree_borders = [(-15, 15), (15, 45), (45, 90), (90, 135), (135, 180),
                          (-180, -135), (-135, -90), (-90, -45), (-45, -15)]

        distance = np.sqrt(x ** 2 + y ** 2)
        if distance > 0.9:
            state[0] = 4

        else:
            for i, (down, up) in enumerate(dis_borders):
                if distance >= down and distance < up:
                    state[0] = i
                    break

        degree = math.atan2(y, x) * 180 / math.pi
        # print 'degree = ', degree

        # (-180, 180]
        if degree <= -180:
            degree += 360
        elif degree > 180:
            degree -= 360
        # print 'degree2 =', degree

        for i, (down, up) in enumerate(degree_borders):
            if degree > down and degree <= up:
                state[1] = i
                break

        # print 'COL ring, fan: ', state
        return state

    def coor_trans(self, command, current_state):
        ''' Gets the command under world frame.
        Input: cmd, under base_link frame: Twist(); current_state, under world frame: ModelState()
        Return: transferred_cmd: Twist()
        '''

        trans_cmd = Twist()

        # step1: Position: x, y
        q = current_state.pose.orientation

        if command.linear.x != 0:
            # 旋转后base_link坐标系在map坐标系中的姿态 × 目标点在base_link坐标系的中的位置 + base_link坐标系在map坐标系中的位置
            r_matrix = trans.quaternion_matrix([q.x, q.y, q.z, q.w])[: 3, : 3]

            t = np.dot(r_matrix, [command.linear.x, command.linear.y, command.linear.z]) + \
                [current_state.pose.position.x, current_state.pose.position.y,
                    current_state.pose.position.z]

            # 参考四元数的变换： result = quaternion_multiply(quaternion_multiply(rot, vec), quaternion_conjugate(rot)) + [[t.x], [t.y], [t.z], [0]]

            trans_cmd.linear.x = t[0]
            trans_cmd.linear.y = t[1]
            trans_cmd.linear.z = current_state.pose.position.z
        else:
            trans_cmd.linear.x = current_state.pose.position.x
            trans_cmd.linear.y = current_state.pose.position.y
            trans_cmd.linear.z = current_state.pose.position.z

        # step 2: angle of z axis
        rpy = trans.euler_from_quaternion([q.x, q.y, q.z, q.w])
        trans_cmd.angular.z = rpy[2] + command.angular.z
        # if out of range [-pi, +pi]
        if trans_cmd.angular.z > math.pi:
            trans_cmd.angular.z -= 2 * math.pi
        elif trans_cmd.angular.z < - math.pi:
            trans_cmd.angular.z += 2 * math.pi

        return trans_cmd

    def get_action_from_index(self, action_index):
        '''get action from action table by action_index
        '''
        cmd = Twist()
        # action pair: (x(meter), yaw(degree))
        action_table = [(0.2, 0), (0.1, 0), (0.05, 0), (0, 0), (0, -30),
                        (0, -10), (0, -3), (0, 3), (0, 10), (0, 30)]
        action = action_table[action_index]
        cmd.linear.x = action[0]
        cmd.angular.z = action[1] * math.pi / 180  # from degree to radian

        return cmd

    def get_reward_from_obs_TGT(self, observation, out_flag):
        '''input: the observation of target seek task
        output: reward and is_terminal flag
        '''
        reward = 0.0
        is_terminal = False

        ring_index_old = self.reward_obs_term['TGT'][1] / \
            self.maze_dim_TGT['num_fan']
        ring_index_new = None

        fan_index_old = self.reward_obs_term['TGT'][1] % self.maze_dim_TGT['num_fan']
        fan_index_new = None

        # step 0: Determine if the agent is out of the border
        if out_flag == True:
            reward = -10.0

        # step 1: Determine if the position gets closer
        if reward == 0:
            ring_index_new = observation / self.maze_dim_TGT['num_fan']
            if ring_index_new < ring_index_old:  # move closer
                reward += 5.0
            elif ring_index_new > ring_index_old:
                reward -= 5.0

            # step 2: Determine if the angle gets closer
            fan_index_new = observation % self.maze_dim_TGT['num_fan']

            # 状态1和状态10到状态0的距离都为1
            max_fan_index = self.maze_dim_TGT['num_fan']
            if min((fan_index_new - 0), (max_fan_index - fan_index_new)) < min((fan_index_old - 0), (max_fan_index - fan_index_old)):
                reward += 5.0
            elif min((fan_index_new - 0), (max_fan_index - fan_index_new)) > min((fan_index_old - 0), (max_fan_index - fan_index_old)):
                reward -= 5.0

        # step 3: Determine if the terminal condition has been met.
        dist = np.sqrt((self.current_state.pose.position.x -
                        self.target_position[0])**2 + (self.current_state.pose.position.y - self.target_position[1])**2)
        if dist <= self.end_radius:
            reward = 50.0
            is_terminal = True
            print 'Succeed!!!!!'

        # step 4： give -1 when no other reward is given, make sure the agent will find the shortest path.
        if reward == 0:
            reward = -1

        return reward, is_terminal

    def get_reward_from_obs_COL(self, observation):
        '''input: the observation of target seek task
        output: reward and is_terminal flag
        '''
        reward = 0.0
        is_terminal = False

        ring_index_old = self.reward_obs_term['COL'][1] / \
            self.maze_dim_COL['num_fan']
        ring_index_new = observation / self.maze_dim_COL['num_fan']

        fan_index_old = self.reward_obs_term['COL'][1] % self.maze_dim_COL['num_fan']
        fan_index_new = observation % self.maze_dim_COL['num_fan']

        # 上一个状态在最外圈，则不对最外圈的状态进行更新
        if ring_index_old == 4:
            pass
        else:
            # 飞到更外圈，+5
            if ring_index_new > ring_index_old:
                reward += 10
            elif ring_index_new < ring_index_old:
                reward -= 5

            # 从内圈脱离范围，加很多
            if ring_index_old < 4 and ring_index_new == 4:
                reward += 20

            # 状态1和状态8到状态0的距离都为1. 前方原理障碍物， +5； 否则-5
            # max_fan_index = self.maze_dim_COL['num_fan']
            # if min((fan_index_new - 0), (max_fan_index - fan_index_new)) < min((fan_index_old - 0), (11 - fan_index_old)):
            #     reward -= 5.0
            # elif min((fan_index_new - 0), (11 - fan_index_new)) > min((fan_index_old - 0), (11 - fan_index_old)):
            #     reward += 5.0

            if fan_index_new >= 3 and fan_index_new <= 6:
                reward += 0
            elif fan_index_new == 2 or fan_index_new == 7:
                reward -= 1
            else:
                reward -= 5

            # in the innerest circle
            if observation <= 7:
                reward = -100
                is_terminal = True
                print 'Crush!!! No!!!'

        return reward, is_terminal

    def env_step(self, action_index):
        """A step taken by the environment.

        Args:
            action: The action taken by the agent

        Returns:
            (float, state, Boolean): a tuple of the reward, state observation,
                and boolean indicating if it's terminal.
        """

        # self.target_position = (
        #     self.target_x, self.target_y, self.init_robot_state.pose.position.z)
        self.obstacle_position = (
            self.obstacle_x, self.obstacle_y, self.init_robot_state.pose.position.z)

        # print "obstacle's position:", self.obstacle_position

        # step 1: get action

        # a = raw_input("请输入一个介于[0, 9]之间的数字,对应9个动作,按c让agent自己选择, 按q退出: ")
        # if a >= '0' and a <= '9':
        #     inta = int(a)
        #     if inta <= 9 and inta >= 0:
        #         action_index = int(a)
        # elif a == 'q':
        #     quit()

        # 在避障状态的才算
        if self.reward_obs_term['COL'][1] < 36 and action_index > 2:
            self.fuck_rotate += 1

        # if action_index > 2:
        #     self.fuck_rotate += 1

        if self.fuck_rotate > 8:
            action_index = 1
            self.fuck_rotate = 0
            print 'fuck rotation!!!!!!!'

        cmd = self.get_action_from_index(action_index)

        # step 2: get the command under the map frame
        cmd_transferred = self.coor_trans(cmd, self.current_state)

        # step 3: Determine if the 'out of border' condition has been met and takes the action.
        out_flag = False
        # if self.out_of_bounds(cmd_transferred, self.current_state, self.target_position, self.reward_obs_term['TGT'][1]):
        #     out_flag = True
        # else:
        #   print 'cmd_transferred =', cmd_transferred
        #   print 'action_index =', action_index
        self.send_cmd_until_stop(cmd_transferred, transfer_flag=False)

        # record every step
        self.trajectory.append(
            (self.current_state.pose.position.x, self.current_state.pose.position.y))

        # =========== TGT mission ==================
        # step4: get the observation (RL state) based on the relative position
        observation = self.get_observation_TGT(
            self.current_state, self.target_position)

        # step5: get reward and is_terminal flag
        reward, is_terminal = self.get_reward_from_obs_TGT(
            observation, out_flag)

        self.reward_obs_term['TGT'] = [reward, observation, is_terminal]

        # =========== COL mission ===================
       # step4: get the observation (RL state) based on the relative position
        observation_COL = self.get_observation_COL(
            self.current_state, self.obstacle_position)

        # step5: get reward and is_terminal flag
        reward_COL, is_terminal_COL = self.get_reward_from_obs_COL(
            observation_COL)

        # 只让到达目标点才结束
        if is_terminal == True:
            is_terminal_COL = True
        else:
            is_terminal_COL = False

        self.reward_obs_term['COL'] = [
            reward_COL, observation_COL, is_terminal_COL]

        # print 'current position', self.current_state.pose.position
        # print "reward_obs_term['COL'] =", self.reward_obs_term['COL']
        return self.reward_obs_term['COL']

    def env_cleanup(self):
        """Cleanup done after the environment ends"""
        path = "/home/ljj/gazebo_drone_tutorials/catkin_ws/src/tabular_dyna_q/scripts/results/"
        io.savemat(path+self.robot_name+'_COL_trajectory.mat',
                   {self.robot_name: self.trajectory})

    def env_message(self, message):
        """A message asking the environment for information

        Args:
            message (string): the message passed to the environment

        Returns:
            string: the response (or answer) to the message
        """
        if message == "what is the current reward?":
            return "{}".format(self.reward_obs_term['TGT'][0])

        # else
        return "I don't know how to respond to your message"


if __name__ == "__main__":
    env = GazeboEnvironment2()

    # command = Twist()
    # command.angular.z = -90 * math.pi / 180  # from radians to degrees
    # command.linear.x = -0.6
    env_info = {"end_radius": 0.05, "target_x": 1.2,
                "target_y": 1.2, 'obstacle_x': 0.6, 'obstacle_y': 0.6}
    env.env_init(env_info)
    env.env_start()
    while True:
        # input() 在Python2和Python3中的含义不同
        t = raw_input("请输入一个介于[0, 9]之间的数字,按q退出: ")
        if t == 'q':
            break
        else:
            a_index = int(t)
        env.env_step(a_index)
