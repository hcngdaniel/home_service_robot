#!/usr/bin/env python3
from .snips import SnipsNLU
from .dataset import Dataset
from collections import namedtuple


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
    def __init__(self, assistant, intent_name, format):
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

    def __call__(self, intent_result):
        formats = {}
        for _ in self.slot_names:
            formats[_] = Slot(intent_result["slots"][[__["slotName"] == _ for __ in intent_result["slots"]].index(True)])
        return self.format.format(**formats)


class Assistant:
    def __init__(self, min_strict):
        self.nlu = SnipsNLU()
        self.dataset = Dataset()
        self.responses = []
        self.min_strict = min_strict

    def set_response(self, intent_name, format):
        self.responses.insert(0, Response(self, intent_name, format))

    def set_dataset(self, dataset: Dataset):
        self.nlu.fit_dataset(dataset.json)
        self.dataset = dataset

    def __call__(self, text, callback=None):
        result = self.nlu.parse(text)
        if result["intent"]["probability"] < self.min_strict:
            return "Sorry, I cannot understand"
        for _ in self.responses:
            if result["intent"]["intentName"] == _.intent_name:
                if callback is not None:
                    callback(result)
                return _(result)
        raise ReferenceError("Response about this intent hasn't been set yet")
