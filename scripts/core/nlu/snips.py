#!/usr/bin/env python3
import os
import io
import json
from snips_nlu import SnipsNLUEngine
from snips_nlu.default_configs import CONFIG_EN

class SnipsNLU:
    def __init__(self):
        self.engine = SnipsNLUEngine(config=CONFIG_EN)
    
    def load(self, engine_path):
        self.engine = SnipsNLUEngine.from_path(engine_path)
    
    def save(self, engine_path):
        self.engine.persist(engine_path)
    
    @staticmethod
    def yaml_to_json(yaml_path, json_path):
        os.system(f"snips-nlu generate-dataset en {yaml_path} > {json_path}")
    
    def fit_dataset(self, dataset_path):
        with io.open(dataset_path) as f:
            self.engine.fit(json.load(f))
    
    def parse(self, text):
        data = self.engine.parse(text)
        return data

