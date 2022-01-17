import yaml
print(yaml.dump_all([{"name": "asdf", "value": {"kind": "None", "value": "test"}}, {"name": "qwer"}, {"name": "uiop"}], indent=2))