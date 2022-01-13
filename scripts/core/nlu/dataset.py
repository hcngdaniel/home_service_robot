#!/usr/bin/env python3
import snips_nlu.dataset as ds
from snips_nlu.common.utils import unicode_string
import io


class Dataset(ds.Dataset):
    def __init__(self, intents = [], entites = []):
        super(Dataset, self).__init__("en", intents, entites)

    def from_yaml(self, filename):
        with io.open(filename) as f:
            ret = ds.Dataset.from_yaml_files("en", [f])
        return ret
