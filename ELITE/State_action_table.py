import numpy as np
import pandas as pd
import math
import fuzzy.FuzzyRouting as FR
import random
import traffic as RNTFA
import fuzzy.AExactData as Exact
import fuzzy.FuzzyRules as rules
import fuzzy.FuzzyRouting as FuzR
import Global_Par as Gp

# State action table

class State_Action_Table:
    def __init__(self):
        self.SAtable = self.initial()

    # State action table initialization
    def initial(self):
        table = {'Security':{'A':[1,0,0],'B':[1,0,0],'C':[1,0,0]}, 'Efficiency':{'A':[1,1,0],'B':[1,1,0],'C':[1,1,0]}, 'Information':{'A':[1,0,0],'B':[0,0,1],'C':[0,0,1]}, 'Entertainment':{'A':[0,0,1],'B':[0,0,1],'C':[0,0,1]}}
        return table

    # State action table maintenance
    def update(self, area_path, trans_type, L, type, ad, hc, rc, ad_list, hc_list, rc_list):
        # 路口内
        if len(area_path) == 1:
            return
        # 相邻路口
        elif len(area_path) == 2:
            # i = area_path[0]
            # j = area_path[1]
            # self.table[i][j][j] += 1
            return
        # AD, HC, RC
        if trans_type == 1:
            alpha = 0.25
            beta = 0
            gamma = 0.25
        elif trans_type == 2:
            alpha = 0.5
            beta = 0.25
            gamma = 0
        elif trans_type == 3:
            alpha = 0.25
            beta = 0.25
            gamma = 0.5
        else:
            print("error")
            print(trans_type)
            exit(0)
        # calculate features
        AD_ = sum(ad_list) / len(ad_list)
        HC_ = sum(hc_list) / len(hc_list)
        RC_ = sum(rc_list) / len(rc_list)
        # update
        R = alpha * AD_ + beta * HC_ + gamma * RC_
        self.SAtable[type][L][trans_type-1] += R
        print(self.SAtable)
        return

