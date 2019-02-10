# -*- coding: utf-8 -*-

import glob
from collections import defaultdict

def get_score(path):
    with open(path) as fin:
        name2score = {}
        for line in fin.readlines():
            if "SCORES" in line:
                tokens = line.split(":")[1].split("|")
                for token in tokens:
                    name, score = tuple(token.strip().split(" "))
                    score = int(score)
                    name2score[name] = score
                return name2score

result_score = defaultdict(int)
path_list = glob.glob("**/stdout", recursive=True)
for path in path_list:
    if "/2/400/" not in path:
        scores = get_score(path)
        for key in scores:
            result_score[key] += scores[key]

for key in result_score:
    print("{}: {}".format(key, result_score[key]))