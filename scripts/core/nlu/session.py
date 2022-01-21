#!/usr/bin/env python3
import typing
from snips_nlu.dataset.intent import Intent
from .word_dict import random_dict
import random


class NotRequestedError(Exception):
    pass


class Session:
    def __init__(self, assistant) -> None:
        self.assistant = assistant
        self.parse_result: typing.Dict = None

    def set_slot(self, slot_name: typing.AnyStr, value: typing.AnyStr, callback: typing.Callable = None) -> typing.Dict:
        if len(value.strip().split(" ")) == 1:
            value = " ".join(random.choices(random_dict, k=3)) + " " + value
        if self.parse_result is None:
            raise NotRequestedError("please request first")
        try:
            slot = self.assistant.nlu.engine.get_slots(text=value, intent="ENTITY" + self.__find_entity_by_slot_name(slot_name))[0]
        except:
            raise ValueError("cannot regonize value")
        slot["range"]["start"], slot["range"]["end"] = float("nan"), float("nan")
        slot["slotName"] = slot_name
        self.parse_result["slots"].append(slot)
        if callable(callback):
            callback(self)
        return self

    def __find_entity_by_slot_name(self, slot_name) -> typing.AnyStr:
        return self.expected_intent.slot_mapping[slot_name]

    def request(self, text: typing.AnyStr, callback: typing.Callable = None) -> typing.Dict:
        self.parse_result = self.assistant.nlu.parse(text)
        if callable(callback):
            callback(self)
        return self

    @property
    def intent_name(self) -> typing.AnyStr:
        if self.parse_result is None:
            raise NotRequestedError("please request first")
        return self.parse_result["intent"]["intentName"]

    @property
    def expected_intent(self) -> Intent:
        for intent in self.assistant.dataset.intents:
            if intent.intent_name == self.intent_name:
                return intent
        raise RuntimeError("intent not found, so weird")

    @property
    def expected_slots(self) -> typing.List[typing.AnyStr]:
        if self.parse_result is None:
            raise NotRequestedError("please request first")
        return self.expected_intent.slot_mapping.keys()

    @property
    def missing_slots(self) -> typing.List[typing.AnyStr]:
        if self.parse_result is None:
            raise NotRequestedError("please request first")
        return list(set(self.expected_slots).difference(set([slot["slotName"] for slot in self.parse_result["slots"]])))
