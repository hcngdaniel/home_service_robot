#!/usr/bin/env python3
import numpy as np
import yaml


user = "Alex"
eps = 1e-10

user_file = f'Users/{user}.yaml'
with open(user_file, 'r') as file:
    user_yaml = dict(yaml.safe_load(file))
user_read = user_yaml['read']
del user_yaml['read']
user_read = map(lambda x: x.lower(), user_read)
user_sum = sum(user_yaml.values())
user_poss = {k: v / user_sum + eps for k, v in user_yaml.items()}

public_file = 'Users/Public.yaml'
with open(public_file, 'r') as file:
    public_yaml = dict(yaml.safe_load(file))
public_sum = sum(public_yaml.values())
public_poss = {k: v / public_sum + eps for k, v in public_yaml.items()}

for k in user_poss.keys():
    user_poss[k] = (user_poss[k] * public_poss[k]) / \
                   (user_poss[k] * public_poss[k] + (1 - user_poss[k]) * (1 - public_poss[k]))

categories = sorted(user_poss.items(), key=lambda x: x[1], reverse=True)

book_list = []

with open(f'Categories/{categories[0][0]}.txt', 'r') as f:
    i = 0
    while i < 3:
        book_name = f.readline().strip()
        if book_name.lower() in user_read:
            continue
        book_list.append(book_name)
        i += 1

with open(f'Categories/{categories[1][0]}.txt', 'r') as f:
    i = 0
    while i < 2:
        book_name = f.readline().strip()
        if book_name.lower() in user_read:
            continue
        book_list.append(book_name)
        i += 1

with open(f'Categories/{categories[2][0]}.txt', 'r') as f:
    i = 0
    while i < 1:
        book_name = f.readline().strip()
        if book_name.lower() in user_read:
            continue
        book_list.append(book_name)
        i += 1

print(book_list)
