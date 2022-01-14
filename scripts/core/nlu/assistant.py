#!/usr/bin/env python3
from .snips import SnipsNLU
from .dataset import Dataset
from collections import namedtuple
from snips_nlu.dataset import Dataset as DS
import tarfile
import time
import io
import json
import pickle


class Slot:
    def __init__(self, in_dict):
        Range = namedtuple("Range", ["start", "end"])
        self.range = Range(start=in_dict["range"]["start"], end=in_dict["range"]["end"])
        self.rawValue = in_dict["rawValue"]
        Value = namedtuple("Value", ["kind", "value"])
        self.value = Value(kind=in_dict["value"]["kind"], value=in_dict["value"]["value"])
        self.entity = in_dict["entity"]
        self.slotName = in_dict["slotName"]


class Response:
    def __init__(self, assistant, intent_name, format, callback=None):
        self.assistant = assistant
        self.intent_name = intent_name
        try:
            assert self.intent_name in [_.intent_name for _ in self.assistant.dataset.intents]
        except AssertionError:
            raise(AttributeError(f"No intent named {self.intent_name} found in Dataset"))
        self.intent_index = [_.intent_name == self.intent_name for _ in self.assistant.dataset.intents].index(True)
        self.slots = self.assistant.dataset.intents[self.intent_index].slot_mapping
        self.slot_names = [_ for _ in self.slots.keys()]
        self.format = format
        self.callback = callback

    def __call__(self, intent_result):
        formats = {}
        for name in self.slot_names:
            try:
                formats[name] = Slot(intent_result["slots"][[slot["slotName"] == name for slot in intent_result["slots"]].index(True)])
            except ValueError:
                formats[name] = None
        if self.callback is not None:
            self.callback(intent_result)
        return self.format.format(**formats)


class Assistant:
    def __init__(self, min_strict):
        self.nlu = SnipsNLU()
        self.dataset = Dataset()
        self.responses = []
        self.min_strict = min_strict
        self._response_intent_names = []
        self._response_formats = []
        self._response_callbacks = []

    def set_response(self, intent_name, format, callback=None):
        self._response_intent_names.append(intent_name)
        self._response_formats.append(format)
        self._response_callbacks.append(callback)
        self.responses.insert(0, Response(self, intent_name, format, callback))

    def save(self, path):
        with tarfile.open(path, "w") as f:
            obj = io.BytesIO(self.nlu.engine.to_byte_array())
            tarinfo = tarfile.TarInfo(name="engine")
            tarinfo.size, tarinfo.mtime = len(obj.getvalue()), time.time()
            f.addfile(tarinfo, fileobj=obj)

            obj = io.BytesIO(pickle.dumps(self._response_intent_names))
            tarinfo = tarfile.TarInfo(name="response_intent_names")
            tarinfo.size, tarinfo.mtime = len(obj.getvalue()), time.time()
            f.addfile(tarinfo, fileobj=obj)

            obj = io.BytesIO(pickle.dumps(self._response_formats))
            tarinfo = tarfile.TarInfo(name="response_formats")
            tarinfo.size, tarinfo.mtime = len(obj.getvalue()), time.time()
            f.addfile(tarinfo, fileobj=obj)

            obj = io.BytesIO(pickle.dumps(self._response_callbacks))
            tarinfo = tarfile.TarInfo(name="response_callbacks")
            tarinfo.size, tarinfo.mtime = len(obj.getvalue()), time.time()
            f.addfile(tarinfo, fileobj=obj)

            config = {}
            config["min_strict"] = self.min_strict
            obj = io.BytesIO(json.dumps(config, indent=4).encode("utf-8"))
            tarinfo = tarfile.TarInfo(name="config.json")
            tarinfo.size, tarinfo.mtime = len(obj.getvalue()), time.time()
            f.addfile(tarinfo, fileobj=obj)

            obj = io.BytesIO(self.dataset.yaml.encode("utf-8"))
            tarinfo = tarfile.TarInfo(name="dataset.yaml")
            tarinfo.size, tarinfo.mtime = len(obj.getvalue()), time.time()
            f.addfile(tarinfo, fileobj=obj)

    @classmethod
    def load(cls, path):
        with tarfile.open(path, "r") as f:
            IO = f.extractfile(f.getmember("engine"))
            engine = IO.read()

            IO = f.extractfile(f.getmember("response_intent_names"))
            _response_intent_names = pickle.load(IO)

            IO = f.extractfile(f.getmember("response_formats"))
            _response_formats = pickle.load(IO)

            IO = f.extractfile(f.getmember("response_callbacks"))
            _response_callbacks = pickle.load(IO)

            IO = f.extractfile(f.getmember("config.json"))
            config = json.load(IO)

            IO = f.extractfile(f.getmember("dataset.yaml"))
            dataset = IO.read().decode()

            ret = Assistant(config["min_strict"])
            ret.nlu.engine.from_byte_array(engine)
            ds = DS.from_yaml_files("en", [io.StringIO(dataset)])
            ret.dataset = Dataset(ds.intents, ds.entities)
            for intent_name, format, callback in zip(_response_intent_names, _response_formats, _response_callbacks):
                ret.set_response(intent_name, format, callback)

            return ret

    def set_dataset(self, dataset: Dataset):
        self.nlu.fit_dataset(dataset.json)
        self.dataset = dataset

    def __call__(self, text):
        result = self.nlu.parse(text)
        if result["intent"]["probability"] < self.min_strict:
            return "Sorry, I cannot understand"
        for response in self.responses:
            if result["intent"]["intentName"] == response.intent_name:
                return response(result)
        raise ReferenceError("Response about this intent hasn't been set yet")
