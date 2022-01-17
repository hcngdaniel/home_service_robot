#!/usr/bin/env python3
import snips_nlu
from .snips import SnipsNLU
from .dataset import Dataset
from snips_nlu.dataset import Dataset as DS
import tarfile
import time
import io
import json
import pickle
from .session import Session


class Assistant:
    def __init__(self):
        self.nlu = SnipsNLU()
        self.dataset = Dataset()
        self.session = Session(self)

    def set_dataset(self, dataset: Dataset):
        self.nlu.fit_dataset(dataset.json)
        self.dataset = dataset

    def save(self, path):
        with tarfile.open(path, "w") as f:
            # engine
            obj = io.BytesIO(pickle.dumps(self.nlu.engine.to_byte_array()))
            tarinfo = tarfile.TarInfo(name="engine")
            tarinfo.size = len(obj.getvalue())
            f.addfile(tarinfo, fileobj=obj)

            # dataset.yaml
            obj = io.BytesIO(self.dataset.yaml.encode("utf-8"))
            tarinfo = tarfile.TarInfo(name="dataset_yaml")
            tarinfo.size = len(obj.getvalue())
            f.addfile(tarinfo, fileobj=obj)

            # dataset.entity_intent_yaml
            obj = io.BytesIO(self.dataset.entity_intent_yaml.encode("utf-8"))
            tarinfo = tarfile.TarInfo(name="entity_intent_yaml")
            tarinfo.size = len(obj.getvalue())
            f.addfile(tarinfo, fileobj=obj)

    @classmethod
    def load(cls, path):
        ret = cls()
        with tarfile.open(path, "r") as f:
            # engine
            IO = f.extractfile(f.getmember("engine"))
            ret.nlu.engine = snips_nlu.SnipsNLUEngine.from_byte_array(pickle.loads(IO.read()))

            # dataset
            IO = f.extractfile(f.getmember("dataset_yaml"))
            dataset_yaml = io.StringIO(IO.read().decode())

            IO = f.extractfile(f.getmember("entity_intent_yaml"))
            entity_intent_yaml = io.StringIO(IO.read().decode())

            ret.dataset = DS.from_yaml_files("en", [dataset_yaml, entity_intent_yaml])
            ret.dataset = Dataset(ret.dataset.intents, ret.dataset.entities)
        return ret
