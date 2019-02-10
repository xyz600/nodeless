# -*- coding: utf-8 -*-

import glob
import sys
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
root_dir = sys.argv[1]
path_list = glob.glob("{}/**/stdout".format(root_dir), recursive=True)
for path in path_list:
    scores = get_score(path)
    for key in scores:
        result_score[key] += scores[key]

for key in result_score:
    print("{}: {}".format(key, result_score[key]))