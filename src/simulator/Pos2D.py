#!/usr/bin/env python

import numpy as np

class Pos2D:
    def __init__(self, _x=0, _y=0, _t=0):
        self.x = _x
        self.y = _y
        self.theta = _t

def add_noise(num=0):
    return num + float(np.random.normal(0, 0.2, 1))
