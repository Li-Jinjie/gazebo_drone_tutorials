#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-08 13:37:17
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-08 13:56:21
@Units        : None
@Description  : file content
@Dependencies : None
@NOTICE       : None
'''

import numpy as np
from scipy.stats import sem
import matplotlib.pyplot as plt
from scipy import io


if __name__ == "__main__":
    # 加载文件
    all_reward_sums = {}
    all_reward_sums['Q-learning'] = np.load('results/q_learning.npy')
    io.savemat('results/q_learning.mat',
               {'q_learning': all_reward_sums['Q-learning']})
    all_reward_sums['Expected Sarsa'] = np.load('results/expected_sarsa.npy')
    io.savemat('results/expected_sarsa.mat', {
               'expected_sarsa': all_reward_sums['Expected Sarsa']})

    for algorithm in ["Q-learning", "Expected Sarsa"]:
        plt.plot(np.mean(all_reward_sums[algorithm], axis=0), label=algorithm)
    plt.xlabel("Episodes")
    plt.ylabel("Sum of\n rewards\n during\n episode", rotation=0, labelpad=40)
    num_episodes = 5
    plt.xlim(0, num_episodes-1)
    # plt.ylim(-100, 0)
    plt.legend(loc=4)
    plt.show()

    # for algorithm, position in [("Q-learning", 211), ("Expected Sarsa", 212)]:
    #     plt.subplot(position)
    #     average_state_visits = np.array(
    #         all_state_visits[algorithm]).mean(axis=0)
    #     grid_state_visits = average_state_visits.reshape((4, 12))
    #     grid_state_visits[0, 1:-1] = np.nan
    #     plt.pcolormesh(grid_state_visits, edgecolors='gray', linewidth=2)
    #     plt.title(algorithm)
    #     plt.axis('off')
    #     cm = plt.get_cmap()
    #     cm.set_bad('gray')

    #     plt.subplots_adjust(bottom=0.0, right=0.7, top=1.0)
    #     cax = plt.axes([0.85, 0.0, 0.075, 1.])
    # cbar = plt.colorbar(cax=cax)
    # cbar.ax.set_ylabel("Visits during\n the last 10\n episodes",
    #                    rotation=0, labelpad=70)
    # plt.show()
