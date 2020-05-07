#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-04 18:48:56
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-07 14:23:12
@Units        : Meter, radian (if no description)
@Description  : env类，与gazebo联动
@Dependencies : None
@NOTICE       : current_state 存的是机器人在world中的状态，target_position存的是要飞到的位置，RL中的state其实是target_position相对current_state的位置，在get_observation()中可以看到具体使用。
'''

from template_environment import BaseEnvironment

import rospy
from geometry_msgs.msg import Twist
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from gazebo_msgs.msg import ModelState
from gazebo_msgs.msg import ModelStates
import copy
import tf.transformations as trans
import numpy as np
import math


class GazeboEnvironment(BaseEnvironment):
    """Implements the environment for an RLGlue environment

    Note:
        env_init, env_start, env_step, env_cleanup, and env_message are required
        methods.
    """

    # def __init__(self):

    #     # initialize
    #     rospy.init_node('Gazebo_Env', anonymous=False)

    #     # -----------Default Robot State-----------------------
    #     self.robot_name = 'uav1'

    #     self.init_robot_state = ModelState()
    #     self.init_robot_state.model_name = self.robot_name
    #     self.init_robot_state.pose.position.x = 0.0
    #     self.init_robot_state.pose.position.y = 0.0
    #     self.init_robot_state.pose.position.z = 1.5
    #     self.init_robot_state.pose.orientation.x = 0.0
    #     self.init_robot_state.pose.orientation.y = 0.0
    #     self.init_robot_state.pose.orientation.z = 0.0
    #     self.init_robot_state.pose.orientation.w = 1.0
    #     self.init_robot_state.twist.linear.x = 0.
    #     self.init_robot_state.twist.linear.y = 0.
    #     self.init_robot_state.twist.linear.z = 0.
    #     self.init_robot_state.twist.angular.x = 0.
    #     self.init_robot_state.twist.angular.y = 0.
    #     self.init_robot_state.twist.angular.z = 0.
    #     self.init_robot_state.reference_frame = 'world'

    #     # ----------- Parameters in RL env class -----------
    #     self.maze_dim = [11, 11]

    #     origin = self.init_robot_state.pose.position
    #     # coordination of the target point.
    #     self.target_position = (origin.x + 0.5, origin.y + 0.0, origin.z)
    #     # self.end_state = [0, 0]
    #     self.end_radius = 0.05  # meters
    #     # The robot's pose and twist infomation under world frame
    #     self.current_state = ModelState()

    #     reward = None
    #     observation = None
    #     termination = None
    #     self.reward_obs_term = [reward, observation, termination]

    #     # -----------Publisher and Subscriber-------------
    #     self.sub_state = rospy.Subscriber(
    #         '/gazebo/model_states', ModelStates, self.state_callback)

    #     self.set_state = rospy.Publisher(
    #         '/gazebo/set_model_state', ModelState, queue_size=10)

    #     self.pub_command = rospy.Publisher(
    #         '/uav1/pose_cmd', Twist, queue_size=10)

    #     rospy.sleep(2.)
    #     # # What function to call when you ctrl + c
    #     # rospy.on_shutdown(self.shutdown)

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

        # -----------Default Robot State-----------------------
        self.robot_name = 'uav1'

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
        self.maze_dim = [11, 11]

        origin = self.init_robot_state.pose.position
        # coordination of the target point.
        self.target_position = (
            origin.x + env_info["target_x"], origin.y + env_info["target_y"], origin.z)
        # self.end_state = [0, 0]
        self.end_radius = env_info["end_radius"]  # meters
        # The robot's pose and twist infomation under world frame
        self.current_state = ModelState()

        # -----------Publisher and Subscriber-------------
        self.sub_state = rospy.Subscriber(
            '/gazebo/model_states', ModelStates, self.state_callback)

        self.set_state = rospy.Publisher(
            '/gazebo/set_model_state', ModelState, queue_size=10)

        self.pub_command = rospy.Publisher(
            '/uav1/pose_cmd', Twist, queue_size=10)

        rospy.sleep(2.)
        # # What function to call when you ctrl + c
        # rospy.on_shutdown(self.shutdown)

        self.reward_obs_term = [0.0, None, False]

    def state_callback(self, States):
        '''The callback function of Subscriber: sub_state.
        Update the current state of robot 'robot_name'
        '''
        index = States.name.index(self.robot_name)
        self.current_state.model_name = self.robot_name
        self.current_state.pose = States.pose[index]
        self.current_state.twist = States.twist[index]
        self.current_state.reference_frame = 'world'
        # print self.current_state

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
                print 'Arrive at the destination!'
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
        # reset the current state = init_state
        self.set_robot_state(self.init_robot_state)
        self.reward_obs_term[1] = self.get_observation(
            self.current_state, self.target_position)
        return self.reward_obs_term[1]

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

    # check if there is an obstacle at (row, col)
    # def is_obstacle(self, row, col):
    #     if [row, col] in self.obstacles:
    #         return True
    #     else:
    #         return False

    def get_observation(self, robot_state, target_position):
        '''
        give coordinations in continuous space (state), return state_observation.
        In: robot_state, ModelState(); target_position, tuple(3)
        Returns: state_observation, int
        '''
        x_relative = target_position[0] - robot_state.pose.position.x
        y_relative = target_position[1] - robot_state.pose.position.y
        # print 'x_relative', x_relative
        # print 'y_relative', y_relative
        state = self.get_polar_coor(x_relative, y_relative)
        return state[0] * self.maze_dim[1] + state[1]

    def get_polar_coor(self, x, y):
        '''get the polar coordinate of Rectangular coordinate (x,y)
        '''
        state = [0, 0]  # ring, fan
        dis_borders = [(0.0, 0.05), (0.05, 0.1), (0.1, 0.2),
                       (0.2, 0.3), (0.3, 0.4), (0.4, 0.5),
                       (0.5, 0.7), (0.7, 0.9), (0.9, 1.1),
                       (1.1, 1.3), (1.3, 1.5)]
        degree_borders = [(0, 45), (45, 75), (75, 85),
                          (85, 95), (95, 105), (105, 135), (135, 180), (180, 225), (225, 270), (270, 315), (315, 360)]

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

        if degree < 0:
            degree += 360
        elif degree >= 360:
            degree -= 360
        # print 'degree2 =', degree

        for i, (down, up) in enumerate(degree_borders):
            if degree >= down and degree < up:
                state[1] = i
                break

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
            r_matrix = trans.quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]

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
        action_table = [(0.1, 0), (0.05, 0), (0, 0), (-0.1, 0), (0, -30),
                        (0, -10), (0, -3), (0, 3), (0, 10), (0, 30)]
        action = action_table[action_index]
        cmd.linear.x = action[0]
        cmd.angular.z = action[1] * math.pi / 180  # from degree to radian

        return cmd

    def env_step(self, action_index):
        """A step taken by the environment.

        Args:
            action: The action taken by the agent

        Returns:
            (float, state, Boolean): a tuple of the reward, state observation,
                and boolean indicating if it's terminal.
        """
        reward = -1.0
        is_terminal = False

        ring_index_old = None
        ring_index_new = None

        # # step 1: get action
        cmd = self.get_action_from_index(action_index)

        # step 2: get the command under the map frame
        cmd_transferred = self.coor_trans(cmd, self.current_state)

        # step 3: Determine if the condition for the action has been met and take the action.
        if self.out_of_bounds(cmd_transferred, self.current_state, self.target_position, self.reward_obs_term[1]):
            reward = -10.0
        else:
            # print 'cmd_transferred =', cmd_transferred
            print 'action_index =', action_index
            print 'cmd_transferred', cmd_transferred
            self.send_cmd_until_stop(cmd_transferred, transfer_flag=False)

        # step 4: Determine if the terminal condition has been met.
        dist = np.sqrt((self.current_state.pose.position.x -
                        self.target_position[0])**2 + (self.current_state.pose.position.y - self.target_position[1])**2)
        if dist <= self.end_radius:
            reward = 50.0
            is_terminal = True
            print 'Succeed!!!!!'

        self.reward_obs_term = [reward, self.get_observation(
            self.current_state, self.target_position), is_terminal]

        print 'reward_obs_term =', self.reward_obs_term
        return self.reward_obs_term

    def env_cleanup(self):
        """Cleanup done after the environment ends"""
        self.current_state = ModelState()

    def env_message(self, message):
        """A message asking the environment for information

        Args:
            message (string): the message passed to the environment

        Returns:
            string: the response (or answer) to the message
        """
        if message == "what is the current reward?":
            return "{}".format(self.reward_obs_term[0])

        # else
        return "I don't know how to respond to your message"


if __name__ == "__main__":
    env = GazeboEnvironment()

    # command = Twist()
    # command.angular.z = -90 * math.pi / 180  # from radians to degrees
    # command.linear.x = -0.6
    env.env_init()
    env.env_start()
    while True:
        # input() 在Python2和Python3中的含义不同
        t = raw_input("请输入一个介于[0, 9]之间的数字,按q退出: ")
        if t == 'q':
            break
        else:
            a_index = int(t)
        env.env_step(a_index)

    # rospy.spin()

    # env.set_robot_state(env.init_robot_state)
    # env.get_observation(env.current_state, env.target_position)
    # for i in range(10):
    #     x = np.random.rand() * 3 - 1.5
    #     y = np.random.rand() * 3 - 1.5
    #     print 'x,y =', x, y
    #     state = env.get_polar_coor(x, y)
    #     print 'state =', state
    #     print 'state_int =', state[0] * 11 + state[1]

    # try:
    #     env.set_robot_state(env.init_robot_state)
    #     # testObj.publish_cmd(command)
    #     # rate = rospy.Rate(1)  # 10hz
    #     # while not rospy.is_shutdown():
    #     #     command_new = testObj.trans_command(command)
    #     #     testObj.pub_command.publish(command_new)
    #     #     rospy.sleep(5)
    # except rospy.ROSInterruptException:
    #     pass
