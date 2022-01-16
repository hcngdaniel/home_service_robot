#!/usr/bin/env python3
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
        missing_slots = []
        for name in self.slot_names:
            try:
                formats[name] = Slot(intent_result["slots"][[slot["slotName"] == name for slot in intent_result["slots"]].index(True)])
            except ValueError:
                formats[name] = None
                missing_slots.append(name)
        if self.callback is not None:
            self.callback(intent_result, missing_slots, self, self.assistant)
        for name in self.slot_names:
            try:
                formats[name] = Slot(intent_result["slots"][[slot["slotName"] == name for slot in intent_result["slots"]].index(True)])
            except ValueError:
                formats[name] = None
                missing_slots.append(name)
        try:
            return self.format.format(**formats)
        except:
            formats = {key: value for key, value in zip([slot[0] for slot in formats.items()],
                                                        [Slot({
                                                            "range": {
                                                                "start": "",
                                                                "end": ""
                                                            },
                                                            "rawValue": "",
                                                            "value": {
                                                                "kind": "",
                                                                "value": "",
                                                            },
                                                            "entity": "",
                                                            "slotName": slot[0]
                                                        }) if slot[1] is None else slot[1] for slot in formats.items()])}
            return self.format.format(**formats)
