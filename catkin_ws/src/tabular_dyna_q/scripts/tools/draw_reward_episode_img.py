#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-08 13:37:17
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-15 15:18:26
@Units        : None
@Description  : file content
@Dependencies : None
@NOTICE       : None
'''

import numpy as np
from scipy.stats import sem
import matplotlib.pyplot as plt
from scipy import io
import os
import copy


if __name__ == "__main__":
    # 加载文件
    all_reward_sums = []
    # pre_path = os.path.abspath(os.path.dirname(os.getcwd()))  # 获取上一级目录

    path = '/home/ljj/gazebo_drone_tutorials/catkin_ws/src/tabular_dyna_q/scripts/results/DynaQ_r1_e150.npy'
    # path = '/home/ljj/gazebo_drone_tutorials/catkin_ws/src/tabular_dyna_q/scripts/results/expected_sarsa_r5_e100.npy'

    all_reward_sums = np.load(path)
    # io.savemat('results/q_learning.mat',
    #            {'q_learning': all_reward_sums['Q-learning']})

    # path = pre_path + '/results/expected_sarsa_r5_e100.npy'
    # all_reward_sums['Expected Sarsa'] = np.load(path)
    # io.savemat('results/expected_sarsa.mat', {
    #            'expected_sarsa': all_reward_sums['Expected Sarsa']})

    # for algorithm in ["Q-learning", "Expected Sarsa"]:
    #     plt.plot(np.mean(all_reward_sums[algorithm], axis=0), label=algorithm)
    average_reward_sum = all_reward_sums[0]
    # average_reward_sum = np.mean(all_reward_sums, axis=0)
    ave_reward_per_episode = copy.deepcopy(average_reward_sum)
    for i, value in enumerate(average_reward_sum):
        if i > 0:
            ave_reward_per_episode[i] = average_reward_sum[i] - \
                average_reward_sum[i-1]

    # plt.plot(np.mean(all_reward_sums, axis=0), label='Dyna-Q')
    plt.plot(ave_reward_per_episode, label='Dyna-Q')
    plt.xlabel("Episodes")
    plt.ylabel("Sum of\n rewards\n during\n episode", rotation=0, labelpad=40)
    num_episodes = 50
    plt.xlim(0, num_episodes-1)
    plt.ylim(0, 150)
    plt.legend(loc=4)
    plt.show()
