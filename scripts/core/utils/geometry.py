#!/usr/bin/env python3
import numpy as np
import typing


def distance(pt1: typing.Tuple[float, float, float], pt2: typing.Tuple[float, float, float]):
    return np.sqrt(np.sum(np.square([pt1[0] - pt2[0], pt1[1] - pt2[0], pt1[2] - pt2[2]])))
