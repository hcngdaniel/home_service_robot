#!/usr/bin/env python3
import os
import io
import yaml
import typing
from geometry_msgs.msg import Pose, Point, Quaternion


class PoseDict:
    def __init__(self):
        self.dict: typing.Dict[typing.Any, Pose] = {}

    def __setitem__(self, key, value):
        self.dict[key] = value

    def __getitem__(self, key) -> Pose:
        return self.dict[key]

    def __delitem__(self, key):
        del self.dict[key]

    def __str__(self):
        return self.dict.__str__()

    def __repr__(self):
        return self.dict.__repr__()

    def all_to_dict(self) -> typing.Dict:
        ret = {}
        for key, value in self.dict.items():
            ret[key] = {"position": {"x": value.position.x,
                                     "y": value.position.y,
                                     "z": value.position.z},
                        "orientation": {"x": value.orientation.x,
                                        "y": value.orientation.y,
                                        "z": value.orientation.z,
                                        "w": value.orientation.w}}
        return ret

    @classmethod
    def from_dict(cls, d: typing.Dict):
        ret = cls()
        for key, value in d.items():
            l = [Point(*list(value["position"].values())), Quaternion(*list(value["orientation"].values()))]
            pose = Pose(*l)
            ret[key] = pose
        return ret

    def save(self, file: typing.Union[str, bytes, os.PathLike]):
        b = yaml.dump(self.all_to_dict()).encode("utf-8")
        with open(file, "wb") as f:
            f.write(b)

    @classmethod
    def load(cls, file: typing.Union[str, bytes, os.PathLike]):
        with open(file, "rb") as f:
            ret = cls.from_dict(yaml.load(f, Loader=yaml.Loader))
        return ret

    def __bytes__(self):
        return bytes(yaml.dump(self.all_to_dict()).encode("utf-8"))
