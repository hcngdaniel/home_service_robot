#!/usr/bin/env python3
from snips import SnipsNLU
from dataset import Dataset


class Response:
    def __init__(self, assistant, intent_name, format):
        self.assistant = assistant
        self.intent_name = intent_name
        self.intent_index = [_.intent_name == self.intent_name for _ in self.assistant.dataset.intents].index(True)
        self.slots = self.assistant.dataset.intents[self.intent_index].slot_mapping
        self.slot_names = []
        for _ in self.slots:
            self.slot_names.append(_["name"])
        try:
            assert self.intent_name in self.slot_names
        except AssertionError:
            raise(AttributeError(f"No intent named {self.intent_name} found in Dataset"))
        self.format = format

    def call(self, intent_result):
        formats = {}
        for _ in self.slot_names:
            formats[_] = intent_result["slots"][[__["slotName"] == _ for __ in intent_result["slots"]].index(True)]
        return self.format.format(**formats)


class Assistant:
    def __init__(self, min_strict):
        self.nlu = SnipsNLU()
        self.dataset = Dataset()
        self.responses = []
        self.min_strict = min_strict

    def set_response(self, intent_name, format):
        self.responses.insert(0, Response(self, intent_name, format))

    def __call__(self, text):
        result = self.nlu.parse(text)
        if result["intent"]["probability"] < self.min_strict:
            return "Sorry, I cannot understand"
        for _ in self.responses:
            if result["intent"]["intentName"] == _.intent_name:
                return _(result)
        raise ReferenceError("Response about this intent hasn't been set yet")
