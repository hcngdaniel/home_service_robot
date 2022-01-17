#!/usr/bin/env python3
import typing
import snips_nlu.dataset as DS
from snips_nlu.common.utils import unicode_string
import io
import yaml
import random
from .word_dict import random_dict
import copy


class Dataset(DS.Dataset):

    slot_intent_template = {
        "type": "intent",
        "name": "", # ENTITY{entity_name}
        "slots": [
            {
                "name": "", # {entity_name}
                "entity": "" # {eytity_name}
            }
        ],
        "utterances": [] # "{random words} {entity_value} {random words}"
    }

    def __init__(self, intents = [], entites = []):
        super(Dataset, self).__init__("en", intents, entites)

    def from_yaml(self, filename):
        with io.open(filename) as f:
            self.yaml = f.read()
        ds = DS.Dataset.from_yaml_files("en", [io.StringIO(self.yaml)])
        self.entity_intents = []
        for entity in ds.entities:
            entity_intent = copy.deepcopy(self.__class__.slot_intent_template)
            entity_intent["name"] = f"ENTITY{entity.name}"
            entity_intent["slots"][0]["name"] = entity.name
            entity_intent["slots"][0]["entity"] = entity.name
            for utterance in entity.utterances:
                pre_words_count, post_words_count = random.randint(1, 5), random.randint(1, 5)
                entity_intent["utterances"].append(" ".join(random.choices(random_dict, k=pre_words_count))
                                                   + f" [{entity.name}]({utterance.value}) " +
                                                   " ".join(random.choices(random_dict, k=post_words_count)))
            for utterance in entity.utterances:
                pre_words_count, post_words_count = random.randint(1, 5), 0
                entity_intent["utterances"].append(" ".join(random.choices(random_dict, k=pre_words_count))
                                                   + f" [{entity.name}]({utterance.value})")
            for utterance in entity.utterances:
                pre_words_count, post_words_count = 0, random.randint(1, 5)
                entity_intent["utterances"].append(f"[{entity.name}]({utterance.value}) " +
                                                   " ".join(random.choices(random_dict, k=post_words_count)))
            if entity_intent["utterances"] == []:
                continue
            self.entity_intents.append(entity_intent)
        self.entity_intent_yaml: typing.AnyStr = yaml.dump_all(self.entity_intents)
        ds = DS.Dataset.from_yaml_files("en", [io.StringIO(self.yaml), io.StringIO(self.entity_intent_yaml)])
        ret = Dataset(ds.intents, ds.entities)
        ret.yaml = self.yaml
        ret.entity_intent_yaml = self.entity_intent_yaml
        ret.entity_intents = self.entity_intents
        return ret
