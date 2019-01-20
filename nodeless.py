#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
Requirements:
  numpy 1.11.3 or above
  matplotlib 2 or above
  networkx 2.2 or above
"""

import sys
import os
import time
import subprocess
import threading
import shlex
import random
import json
import itertools
import datetime
from optparse import OptionParser

import numpy as np
import matplotlib.pyplot as plt
import networkx as nx

VERSION  = "0.14"
REVISION = "a"
VER_DATE = "20190102"

SHOW_GRAPH = False
SAVE_GRAPH = False
DEBUG_PRINT = False
TIME_LIMIT = 10.0

COLORS = ("lightgray", "r", "g", "lightblue", "m", "c", "y", "k")

def read_response(player):
    start_time = time.time()
    class ExecThread(threading.Thread):
        def __init__(self, solver):
            self.solver = solver
            self.response = None
            threading.Thread.__init__(self)
        def run(self):
            self.response = self.solver.stdout.readline().strip()
        def get_response(self):
            return self.response
    t = ExecThread(player.solver)
    t.setDaemon(True)
    t.start()
    if t.isAlive(): t.join(player.time)
    player.time -= time.time() - start_time
    return t.get_response(), player.time

def check_TLE(player):
    if player.time <= 0.0:
        print("Error: time limit exceeded: %s" % (player.name,), file=sys.stderr)
        return True
    return False

def quit_game(players):
    for player in players:
        try:
            player.solver.stdin.write(("%s\n" % json.dumps({"action": "quit"})).encode("utf-8"))
            player.solver.stdin.flush()
            response, player.times = read_response(player)
        except:
            pass

class Player:
    def __init__(self, uid, name, solver, num_units):
        self.uid = uid
        self.name = name
        self.solver = solver
        self.num_units = num_units
        self.time = TIME_LIMIT
        self.nodes = []

class Nodeless:
    """Nodeless class
    """
    def __init__(self, players, num_nodes=30, edge_prob=0.2, num_units=None, seed=None, config_file=None):
        self.G = None
        if config_file:
            data = json.load(open(config_file))
            self.G = nx.Graph()
            self.G.add_nodes_from(data["nodes"])
            self.G.add_edges_from(data["edges"])
            self.num_nodes = len(self.G.nodes)
        else:
            self.num_nodes = num_nodes
            self.G = nx.fast_gnp_random_graph(self.num_nodes, edge_prob, seed=seed)
        nx.set_node_attributes(self.G, name="player", values=-1)
        nx.set_node_attributes(self.G, name="number", values=0)

        self.num_players = len(players)
        self.players = []
        self.num_units = num_units if num_units and num_units > 0 else max(1, self.num_nodes // (self.num_players * 2))
        for uid, (name, solver) in enumerate(players):
            solver = subprocess.Popen(solver, shell=True, stdin=subprocess.PIPE, stdout=subprocess.PIPE)
            self.players.append(Player(uid, name, solver, self.num_units))
        self.nodeless = []

        print("num_nodes: {}".format(self.num_nodes))
        if config_file:
            print("config_file: {}".format(config_file))
        else:
            print("edge_prob: {}".format(edge_prob))
        print("players: {}".format(", ".join([player.name for player in self.players])))
        print("num_players: {}".format(self.num_players))
        print("num_units: {}".format(self.num_units))
        print("time limit: {}".format(TIME_LIMIT))

        self.trajectories = []

    def judge_init(self, responses):
        U = np.zeros((self.num_nodes, self.num_players), dtype=int)
        for uid, response in responses:
            data = json.loads(response.decode("utf-8"))
            for pos in data["positions"]:
                U[pos][uid] += 1
        for pos, xs in enumerate(U):
            if self.G.node[pos]["player"] >= 0:
                continue
            t = sorted(xs)[-2]
            u = np.asarray(xs, dtype=int) - t
            uid = np.argmax(u)
            if u[uid] > 0:
                self.G.node[pos]["player"] = uid
                self.G.node[pos]["number"] = u[uid]

    def judge_play(self, responses):
        U = np.zeros((self.num_nodes, self.num_players), dtype=int)
        for uid, response in responses:
            if not response:
                continue
            data = json.loads(response.decode("utf-8"))
            src = data["src"]
            dst = data["dst"]
            num = data["num"]
            if self.G.node[src]["player"] != uid:
                print("Error: invalid unit: node (%d) has no uid (%d)" % (src, uid), file=sys.stderr)
                self.nodeless.append(uid)
                continue
            if self.G.node[src]["number"] < num:
                print("Error: invalid unit: node (%d) has no %d unit(s)" % (src, num), file=sys.stderr)
                self.nodeless.append(uid)
                continue
            if self.G.node[src]["number"] == 0:
                print("Error: invalid unit: unit to select is 1 or more", file=sys.stderr)
                self.nodeless.append(uid)
                continue
            if self.G.node[dst]["player"] >= 0:
                print("Error: invalid unit: node (%d) is not free" % (dst,), file=sys.stderr)
                self.nodeless.append(uid)
                continue
            if not self.G.has_edge(src, dst):
                print("Error: invalid unit: node (%d) and (%d) has no edge" % (src, dst), file=sys.stderr)
                self.nodeless.append(uid)
                continue
            self.G.node[src]["number"] -= num
            U[dst][uid] = num
        for pos, xs in enumerate(U):
            if self.G.node[pos]["player"] >= 0:
                continue
            t = sorted(xs)[-2]
            u = np.asarray(xs, dtype=int) - t
            uid = np.argmax(u)
            if u[uid] > 0:
                self.G.node[pos]["player"] = uid
                self.G.node[pos]["number"] = u[uid]

    def judge_graph(self):
        def search_free_nodes(node, memo):
            if self.G.node[node]["player"] == -1:
                memo.append(node)
                for neighbor in self.G.neighbors(node):
                    if neighbor not in memo:
                        search_free_nodes(neighbor, memo)
        groups = []
        for node in self.G.nodes():
            if self.G.node[node]["player"] >= 0 or node in list(itertools.chain.from_iterable(groups)):
                continue
            memo = []
            search_free_nodes(node, memo)
            groups.append(memo)
        if DEBUG_PRINT:
            print("groups: {}".format(groups), file=sys.stderr) ##### debug
        for group in groups:
            boundary = list(nx.algorithms.boundary.node_boundary(self.G, group))
            boundary_players = set([self.G.node[node]["player"] for node in boundary])
            if DEBUG_PRINT:
                print("boundary: {}".format(boundary), file=sys.stderr) ##### debug
                print("boundary_players: {}".format(boundary_players), file=sys.stderr) ##### debug
            if len(boundary_players) == 1:
                p = boundary_players.pop()
                for node in group:
                    self.G.node[node]["player"] = p
        
        for uid in range(self.num_players):
            if uid in self.nodeless:
                continue
            basements = []
            for node in self.G.nodes():
                if self.G.node[node]["player"] == uid and self.G.node[node]["number"] > 0:
                    basements.append(node)
            boundary = list(nx.algorithms.boundary.node_boundary(self.G, basements))
            if -1 not in [self.G.node[node]["player"] for node in boundary]:
                self.nodeless.append(uid)

    def initialize(self):
        names = [player.name for player in self.players]
        responses = []
        for player in self.players:
            try:
                data = json.dumps({"action": "init", "uid": player.uid, "names": names, "num_units": player.num_units, "nodes": list(self.G.nodes()), "edges": list(self.G.edges()), "state": [-1] * self.G.number_of_nodes(), "units": [0] * self.G.number_of_nodes()})
                if DEBUG_PRINT:
                    print("{}: {}".format(player.name, data), file=sys.stderr) ###
                player.solver.stdin.write(("%s\n" % data).encode("utf-8"))
                player.solver.stdin.flush()
                response, player.time = read_response(player)
                responses.append((player.uid, response))
                if check_TLE(player):
                    return False
            except Exception as e:
                print(str(e), file=sys.stderr)
        self.judge_init(responses)
        self.judge_graph()
        self.show_score()
        self.trajectories.append((self.G.copy(), self.get_score()))

        return True
    
    def show_graph(self, savename="result"):
        num_trajectories = len(self.trajectories)
        nr = int(np.ceil(np.sqrt(num_trajectories)))
        fig, ax = plt.subplots(nr, nr)
        fig.set_size_inches(20, 20)
        for i in range(nr):
            for j in range(nr):
                ax[i, j].set_xticks([])
                ax[i, j].set_yticks([])
        for i, (G, (players, free_nodes)) in enumerate(self.trajectories):
            ix = np.unravel_index(i, ax.shape)
            ax[ix].set_title("step={} players={} free_nodes={}".format(i, players, free_nodes), fontsize=10)
            colors = list(map(lambda v: COLORS[v["player"]+1], dict(G.nodes).values()))
            np.random.seed(0)
            nx.draw_networkx(G, pos=nx.spring_layout(G), ax=ax[ix], node_shape="o", node_color=colors, labels={k: v["number"] for k, v in G.node.items()}) ###
        if SAVE_GRAPH:
            plt.savefig("{0:}_{1:%Y%m%d%H%M%S%f}.png".format(savename, datetime.datetime.now()))
        if SHOW_GRAPH:
            plt.show()

    def play(self):
        responses = []
        for player in self.players:
            try:
                response = None
                if player.uid not in self.nodeless:
                    state = [int(self.G.node[node]["player"]) for node in self.G.nodes()]
                    units = [int(self.G.node[node]["number"] if self.G.node[node]["player"] == player.uid else 0) for node in self.G.nodes()]
                    data = json.dumps({"action": "play", "nodes": list(self.G.nodes()), "edges": list(self.G.edges()), "state": state, "units": units})
                    if DEBUG_PRINT:
                        print("{}: {}".format(player.name, data), file=sys.stderr) ##### debug
                    player.solver.stdin.write(("%s\n" % data).encode("utf-8"))
                    player.solver.stdin.flush()
                    response, player.time = read_response(player)
                responses.append((player.uid, response))
                if check_TLE(player):
                    return False
            except Exception as e:
                print(str(e), file=sys.stderr)
                return False
        self.judge_play(responses)
        self.judge_graph()
        self.show_score()
        self.trajectories.append((self.G.copy(), self.get_score()))

        if len(self.nodeless) == self.num_players:
            return False
        return True

    def get_score(self):
        scores = np.zeros(self.num_players, dtype=int)
        free_nodes = 0
        for node in self.G.nodes():
            p = self.G.node[node]["player"]
            if p >= 0:
                scores[p] += 1
            else:
                free_nodes += 1
        return scores, free_nodes

    def show_score(self):
        scores, free_nodes = self.get_score()
        print("step={:04d} players={} free_nodes={}".format(len(self.trajectories), scores, free_nodes))

    def finish(self):
        scores, _ = self.get_score()
        result = []
        for player, score in zip(self.players, scores):
            result.append([score, player.name])
        result.sort(reverse=True)
        rank = self.num_players
        order = self.num_players
        prev_score = -1
        for i in range(len(result)-1, -1, -1):
            if prev_score < result[i][0]:
                rank = order
            prev_score = result[i][0]
            result[i].append(rank)
            order -= 1
        print("### RESULT: {}".format(" | ".join(map(lambda xs: "({}) {} {}".format(xs[2], xs[1], xs[0]), result))))
        SCORE_TABLE = range(self.num_players, 0, -1)
        print("### SCORES: {}".format(" | ".join(map(lambda xs: "{} {}".format(xs[1], SCORE_TABLE[xs[2]-1]), result))))
        quit_game(self.players)

def main():
    global DEBUG_PRINT, SHOW_GRAPH, SAVE_GRAPH, TIME_LIMIT

    parser = OptionParser(usage="Usage: %prog [options] name1 command1 name2 command2 [name3 command3 ...]")
    parser.add_option("-c", "--config", action="store", type="string", dest="config", default=None, help="config file to create graph")
    parser.add_option("--show", action="store_true", dest="show", default=False, help="show graph")
    parser.add_option("--save", action="store_true", dest="save", default=False, help="save graph")
    parser.add_option("-n", "--nodes", action="store", type="int", dest="nodes", default=30, help="number of nodes")
    parser.add_option("-p", "--prob", action="store", type="float", dest="prob", default=0.2, help="edge probability")
    parser.add_option("-u", "--units", action="store", type="int", dest="units", default=None, help="number of units")
    parser.add_option("-t", "--time", action="store", type="float", dest="time", default=10.0, help="time limit")
    parser.add_option("-s", "--seed", action="store", type="int", dest="seed", default=None, help="seed for random number generator")
    parser.add_option("-d", "--debug", action="store_true", dest="debug", default=False, help="print for debug")

    (options, args) = parser.parse_args()

    if len(args) < 4:
        parser.error("incorrect number of arguments")

    DEBUG_PRINT = options.debug
    SHOW_GRAPH = options.show
    SAVE_GRAPH = options.save
    TIME_LIMIT = options.time
    config_file = options.config
    seed = options.seed

    n = len(args) // 2
    players = [(name, solver) for name, solver in zip(args[0:2*n-1:2], args[1:2*n:2])]
    nodeless = Nodeless(players, options.nodes, options.prob, options.units, seed=seed, config_file=config_file) ### default: num_nodes=30, edge_prob=0.2
    if nodeless.initialize():
        while True:
            if not nodeless.play():
                break
        nodeless.finish()
        if SHOW_GRAPH or SAVE_GRAPH:
            nodeless.show_graph()

if __name__ == "__main__":
    main()
