#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""tadashi's Nodelss Solver (C) 2018 Fixstars Corp.
"""

import sys
import os
import random
import json

import networkx as nx

VERSION  = "0.06"
REVISION = "a"
VER_DATE = "20181206"

DEBUG_PRINT = False

def initial_positions(G, num_units):
    return sorted(random.choices(list(G.nodes()), k=num_units))

def solver(G):
    for node, attr in G.node.items():
        if attr["number"] > 0:
            for neighbor in G.neighbors(node):
                if G.node[neighbor]["player"] == -1:
                    return node, neighbor, attr["number"]
    return -1, -1, -1

def main(args):
    G = nx.Graph()

    while True:
        text = sys.stdin.readline().strip()
        if DEBUG_PRINT:
            print("tadashi:", text, file=sys.stderr) ##### debug
        data = json.loads(text)
        action = data["action"]
        if action == "play":
            nodes = data["nodes"]
            edges = data["edges"]
            state = data["state"]
            units = data["units"]
            nx.set_node_attributes(G, name="player", values={node: player for node, player in zip(nodes, state)})
            nx.set_node_attributes(G, name="number", values={node: player for node, player in zip(nodes, units)})
            src, dst, num = solver(G)
            sys.stdout.write("%s\n" % json.dumps({"src": src, "dst": dst, "num": num}))
            if DEBUG_PRINT:
                print("tadashi:", {"src": src, "dst": dst, "num": num}, file=sys.stderr) ##### debug
            sys.stdout.flush()
        elif action == "init":
            uid = data["uid"]
            names = data["names"]
            name = names[uid]
            num_units = data["num_units"]
            nodes = data["nodes"]
            edges = data["edges"]
            state = data["state"]
            units = data["units"]
            G.add_nodes_from(nodes)
            G.add_edges_from(edges)
            nx.set_node_attributes(G, name="player", values={node: player for node, player in zip(nodes, state)})
            nx.set_node_attributes(G, name="number", values=0)
            positions = initial_positions(G, num_units)
            sys.stdout.write("%s\n" % json.dumps({"positions": positions}))
            if DEBUG_PRINT:
                print({"positions": positions}, file=sys.stderr) ##### debug
            sys.stdout.flush()
        elif action == "quit":
            sys.stdout.flush()
            break

if __name__ == "__main__":
    main(sys.argv)
