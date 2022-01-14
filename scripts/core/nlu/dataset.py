#!/usr/bin/env python3
import snips_nlu.dataset as DS
from snips_nlu.common.utils import unicode_string
import io


class Dataset(DS.Dataset):
    def __init__(self, intents = [], entites = []):
        super(Dataset, self).__init__("en", intents, entites)
        self.yaml = ""

    def from_yaml(self, filename):
        with io.open(filename) as f:
            self.yaml = f.read()
            ds = DS.Dataset.from_yaml_files("en", [io.StringIO(self.yaml)])
            ret = Dataset(ds.intents, ds.entities)
            ret.yaml = self.yaml
        return ret
