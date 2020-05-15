#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-08 13:37:17
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-15 15:15:58
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


if __name__ == "__main__":
    # 加载文件
    path = '/home/ljj/gazebo_drone_tutorials/catkin_ws/src/tabular_dyna_q/scripts/results/DynaQ_state_r1_e150.npy'
    tmp = np.load(path)
    # all_state_visits = tmp.item()
    all_state_visits = tmp

    # for algorithm, position in [("Q-learning", 211), ("Expected Sarsa", 212)]:
    #     plt.subplot(position)
    #     average_state_visits = np.array(
    #         all_state_visits[algorithm]).mean(axis=0)
    #     grid_state_visits = average_state_visits.reshape((11, 11))
    #     # grid_state_visits[0, 1:-1] = np.nan
    #     plt.pcolormesh(grid_state_visits, edgecolors='gray', linewidth=2)
    #     plt.title(algorithm)
    #     plt.axis('off')
    #     cm = plt.get_cmap()
    #     cm.set_bad('gray')

    # plt.subplots_adjust(bottom=0.05, right=0.7, top=1.0)
    # cax = plt.axes([0.85, 0.05, 0.075, 0.9])
    # plt.subplot(position)
    # average_state_visits = np.array(
    #     all_state_visits).mean(axis=0)
    grid_state_visits = all_state_visits.reshape((11, 11))
    # grid_state_visits[0, 1:-1] = np.nan
    plt.pcolormesh(grid_state_visits, edgecolors='gray', linewidth=2)
    plt.title('Dyna-Q')
    plt.axis('off')
    cm = plt.get_cmap()
    cm.set_bad('gray')

    # plt.subplots_adjust(bottom=0.05, right=0.7, top=1.0)
    cax = plt.axes([0.85, 0.1, 0.075, 0.8])
    cbar = plt.colorbar(cax=cax)
    # cbar.ax.set_ylabel("Visits during\n the last 10\n episodes",
    #                    rotation=0, labelpad=70)
    cbar.ax.set_ylabel("Visits during\n the last 10\n episodes",
                       rotation=0)
    plt.show()
