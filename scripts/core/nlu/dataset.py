#!/usr/bin/env python3
import snips_nlu.dataset as ds


class Dataset(ds.Dataset):
    def __init__(self, intents = [], entites = []):
        super(Dataset, self).__init__("en", intents, entites)

    def save(self, path):
        with open(path, "w") as f:
            s = json.dumps(self.json, indent=4)
            f.write(s)
