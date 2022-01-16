#!/usr/bin/env python3
import typing
from .assistant import Assistant
from snips_nlu.dataset.intent import Intent


class NotRequestedError(Exception):
    pass


class Session:
    def __init__(self, assistant: Assistant) -> None:
        self.assistant: Assistant = assistant
        self.parse_result: typing.Dict = None

    def set_slot(self, slot_name: typing.AnyStr, value: typing.AnyStr, callback: typing.Callable = None) -> typing.Dict:
        if self.parse_result is None:
            raise NotRequestedError("please request first")
        slot = self.assistant.nlu.engine.get_slots(text=slot_name + " = " + value, intent="SLOT" + slot_name)[0]
        self.parse_result["slots"].append(slot)
        if callable(callback):
            callback(self.parse_result)
        return self.parse_result

    def request(self, text: typing.AnyStr, callback: typing.Callable = None) -> typing.Dict:
        self.parse_result = self.assistant.nlu.parse(text)
        if callable(callback):
            callback(self.parse_result)
        return self.parse_result

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
        raise Exception("intent not found, so weird")

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
