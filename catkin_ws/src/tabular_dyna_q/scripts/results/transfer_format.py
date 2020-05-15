#!/usr/bin/env python
# coding=utf-8
'''
@Author       : LI Jinjie
@Date         : 2020-05-15 11:30:21
@LastEditors  : LI Jinjie
@LastEditTime : 2020-05-15 11:37:38
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
    path = "/home/ljj/gazebo_drone_tutorials/catkin_ws/src/tabular_dyna_q/scripts/results/"
    name = 'DynaQ_table_r2_e50.npy'
    arr = np.load(path+name)

    io.savemat(path+name+'.mat', {'DynaQ_table_r2_e50': arr})
