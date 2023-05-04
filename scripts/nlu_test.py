#!/usr/bin/env python3
import core


assistant = core.nlu.Assistant()
dataset = core.nlu.Dataset().from_yaml("cml_dataset.yaml")
assistant.set_dataset(dataset)

session = assistant.session

for i in range(10000):
    session.request(input())
    print(session.get_slot_value('direction'))

