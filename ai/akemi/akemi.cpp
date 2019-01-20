// akemi's Nodeless Solver (C) 2018 Fixstars Corp.
// g++ akemi.cpp -std=c++14 -o akemi -O3 -Wall -Wno-unused-but-set-variable

#include <iostream>
#include <fstream>
#include <string>
#include <vector>
#include <map>
#include <random>
#include <algorithm>
#include <utility>

#include "json.hpp"

constexpr const char* version  = "0.04";
constexpr const char* revision = "a";
constexpr const char* ver_date = "20181211";

//#define DEBUG_PRINT

void remove_newline(std::string& s)
{
	std::string target("\n");
	std::string::size_type pos = s.find(target);
	while (pos != std::string::npos) {
		s.replace(pos, target.size(), "");
		pos = s.find(target, pos);
	}
}

struct Node {
	std::vector<Node*> neighbors;
    int idx;
	int player;
	int number;
	int value;

    Node(int idx, int player, int number) : idx(idx), player(player), number(number) {
		value = 0;
	}
};

class Graph {
public:
	typedef std::map<int, Node*> nodes_t;
    nodes_t nodes;

    void add_node(const int idx) {
    	nodes_t::iterator it = nodes.find(idx);
	    if (it == nodes.end()) {
    	    Node *node;
        	node = new Node(idx, -1, 0);
        	nodes[idx] = node;
        	return;
    	}
    	std::cerr << "Node already exists!" << std::endl;
	}
    void add_edge(const int from, const int to) {
	    Node* f = (nodes.find(from)->second);
	    Node* t = (nodes.find(to)->second);
		f->neighbors.push_back(t);
		t->neighbors.push_back(f);
	}
	int num_nodes() const { return static_cast<int>(nodes.size()); }
};

std::vector<int> initial_positions(Graph* G, int num_units)
{
	std::vector<int> positions;
	std::vector<std::pair<int, Node*>> priority_nodes;

	for (const auto& node : G->nodes) {
		int num_neighbors = node.second->neighbors.size();
		std::for_each(node.second->neighbors.begin(), node.second->neighbors.end(),
			[num_neighbors](const auto& neighbor){ neighbor->value += 100 / num_neighbors; });
	}
	for (const auto& node : G->nodes) {
		priority_nodes.push_back(std::make_pair(node.second->value, node.second));
	}
	std::sort(priority_nodes.begin(), priority_nodes.end(), [](const auto& a, const auto& b){ return a.first > b.first; });
	for (const auto& node : priority_nodes) {
		positions.push_back(node.second->idx);
		if (static_cast<int>(positions.size()) >= num_units) break;
	}
	int remains = num_units - positions.size();
	for (int i = 0; i < remains; i++) {
		positions.push_back(positions.front());
	}
	std::sort(positions.begin(), positions.end());

#ifdef DEBUG_PRINT
	for (const auto& node : priority_nodes) {
		std::cerr << node.second->value << ", " << node.second->idx << ", " << node.second->neighbors.size() << ", " << node.second->player << std::endl; ///// debug
	}
#endif // DEBUG_PRINT

	return positions;
}

void solver(Graph* G, int& src, int& dst, int& num)
{
	std::vector<std::pair<int, Node*>> priority_nodes;
	for (const auto& node : G->nodes) {
		int value = std::count_if(node.second->neighbors.begin(), node.second->neighbors.end(),
			[](const auto& neighbor){ return neighbor->player < 0; });
		priority_nodes.push_back(std::make_pair(value, node.second));
	}
	std::sort(priority_nodes.begin(), priority_nodes.end(), [](const auto& a, const auto& b){ return a.first > b.first; });

	for (const auto& node : priority_nodes) {
		if (node.second->number > 0 && !node.second->neighbors.empty()) {
			for (const auto& neighbor : node.second->neighbors) {
				if (neighbor->player == -1) {
					src = node.second->idx;
					dst = neighbor->idx;
					num = node.second->number;
					return;
				}
			}
		}
	}
	src = -1;
	dst = -1;
	num = -1;
}

int main(int argc, char* argv[])
{
	std::string s;

	Graph* G = new Graph();

	for (;;) {
		getline(std::cin, s);
		nlohmann::json obj = nlohmann::json::parse(s);
		auto action = obj["action"];
			
		if (action == "play") {
            auto nodes(obj["nodes"]);
            auto edges(obj["edges"]);
            auto state(obj["state"]);
            auto units(obj["units"]);

			for (int i = 0; i < G->num_nodes(); i++) {
				G->nodes[i]->player = state[i];
				G->nodes[i]->number = units[i];
			}

			int src;
			int dst;
			int num;
			solver(G, src, dst, num);

			nlohmann::json out;
			out["src"] = src;
			out["dst"] = dst;
			out["num"] = num;
			std::string json(out.dump());
			remove_newline(json);
			std::cout << json << std::endl << std::flush;

#ifdef DEBUG_PRINT
			std::cerr << "akemi: " << json << std::endl; ///// debug
#endif // DEBUG_PRINT

		} else if (action == "init") {
			int uid(obj["uid"]);
			auto names(obj["names"]);
			auto name(names[uid]);
			auto num_units(obj["num_units"]);
            auto nodes(obj["nodes"]);
            auto edges(obj["edges"]);
            auto state(obj["state"]);
            auto units(obj["units"]);

			for (const auto& node : nodes) {
				G->add_node(node);
			}
			for (const auto& edge : edges) {
				G->add_edge(edge[0], edge[1]);
			}

			std::vector<int> positions(initial_positions(G, num_units));

			nlohmann::json out;
			out["positions"] = positions;
			std::string json(out.dump());
			remove_newline(json);
			std::cout << json << std::endl << std::flush;

		} else if (action == "quit") {
			std::cout << std::endl << std::flush;
			break;
		}
	}

	delete G;
	
	return 0;
}
