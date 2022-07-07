#!/usr/bin/env python3
from abc import ABC
import typing


class BaseSolution(ABC):
    def __init__(self):
        self.name: str = ""
        self.result = None
        self.result_topic: typing.AnyStr = ""

    def __callback__(self, msg):
        raise NotImplementedError()
