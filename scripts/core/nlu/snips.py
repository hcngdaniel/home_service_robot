#!/usr/bin/env python3
import io
import json
from snips_nlu import SnipsNLUEngine
from snips_nlu.default_configs import CONFIG_EN
from snips_nlu.dataset.dataset import Dataset


class SnipsNLU:
    def __init__(self):
        self.engine = SnipsNLUEngine(config=CONFIG_EN)
    
    def fit_dataset(self, data_json):
            self.engine.fit(data_json)
    
    def parse(self, text):
        data = self.engine.parse(text)
        return data
