#include <algorithm>
#include <cstdint>
#include <iostream>
#include <map>
#include <unordered_set>
#include <vector>

#include "json.hpp"

using namespace std;

template <typename N = std::size_t, typename E = std::size_t>
class SparseGraph
{
public:
    using NodeType = N;
    using EdgeType = E;
    using NeighborListType = std::unordered_set<std::size_t>;

    SparseGraph(bool undirected = true) noexcept
        : undirected_(undirected)
    {
    }

    void push_node(NodeType n) noexcept
    {
        nodes_.push_back(n);
        neighbor_list_.emplace_back();
    }

    void connect(const std::size_t from, const std::size_t to, const EdgeType edge = EdgeType()) noexcept
    {
        if (undirected_)
        {
            neighbor_list_[from].insert(to);
            neighbor_list_[to].insert(from);
            edges_[std::make_pair(std::min(from, to), std::max(from, to))] = edge;
        }
        else
        {
            neighbor_list_[from].insert(to);
            edges_[std::make_pair(from, to)] = edge;
        }
    }

    void disconnect(const std::size_t from, const std::size_t to) noexcept
    {
        if (undirected_)
        {
            neighbor_list_[from].erase(to);
            neighbor_list_[to].erase(from);
            edges_.erase(std::make_pair(std::min(from, to), std::max(from, to)));
        }
        else
        {
            neighbor_list_[from].erase(to);
            edges_.erase(std::make_pair(from, to));
        }
    }

    const NodeType& node(const std::size_t index) const noexcept
    {
        return nodes_.at(index);
    }

    const EdgeType& edge(const std::size_t from, const std::size_t to) const noexcept
    {
        if (undirected_)
        {
            auto key = std::make_pair(std::min(from, to), std::max(from, to));
            return edges_.at(key);
        }
        else
        {
            auto key = std::make_pair(from, to);
            return edges_.at(key);
        }
    }

    NodeType& node(const std::size_t index) noexcept
    {
        return nodes_.at(index);
    }

    EdgeType& edge(const std::size_t from, const std::size_t to) noexcept
    {
        if (undirected_)
        {
            auto key = std::make_pair(std::min(from, to), std::max(from, to));
            return edges_.at(key);
        }
        else
        {
            auto key = std::make_pair(from, to);
            return edges_.at(key);
        }
    }

    const NeighborListType&
    neighbor(const std::size_t index) const noexcept
    {
        return neighbor_list_[index];
    }

    std::size_t size() const noexcept
    {
        return neighbor_list_.size();
    }

private:
    using EdgeMap = std::map<std::pair<std::size_t, std::size_t>, EdgeType>;

    std::vector<NeighborListType> neighbor_list_;

    EdgeMap edges_;

    std::vector<NodeType> nodes_;

    bool undirected_;
};

enum NodeState
{
    // ある地点に 1個 Unit を置かれたら即座に持ち主が決定しうる場所
    Closed,
    // Closed なノードのうち、近傍に Closed と Open なノードを含むもの
    Border,
    // Closed でないノード
    Opened,
    // 初期値
    Unknown,
};

struct Node
{
    int PlayerId;
    int UnitCount;
    NodeState state;

    Node()
        : PlayerId(-1)
        , UnitCount(0)
        , state(Unknown)
    {
    }
};

struct Command
{
    int SrcNodeId;
    int DstNodeId;
    int UnitNum;

    Command()
        : SrcNodeId(0)
        , DstNodeId(0)
        , UnitNum(0)
    {
    }

    Command(int src, int dst, int unit)
        : SrcNodeId(src)
        , DstNodeId(dst)
        , UnitNum(unit)
    {
    }
};

SparseGraph<Node> ParseGraph(const nlohmann::json& obj)
{
    SparseGraph<Node> graph;
    const auto node_list = obj["nodes"];
    const auto edge_list = obj["edges"];

    for (auto& node : node_list)
    {
        const size_t nodeId = node;
        graph.push_node(Node());
    }

    for (auto& edge : edge_list)
    {
        const size_t from = edge[0];
        const size_t to = edge[1];
        graph.connect(from, to);
    }

    auto states = obj["state"];
    for (size_t i = 0; i < states.size(); i++)
    {
        graph.node(i).PlayerId = states[i];
        graph.node(i).UnitCount = 1;
    }

    return graph;
}

vector<size_t> SelectUnitList(SparseGraph<Node>& graph, const size_t unit_size)
{
    vector<size_t> ret;

    int id = 0;
    for (size_t iter = 0; iter < unit_size; iter++)
    {
        ret.push_back(id);
        id = (id == graph.size() - 1 ? 0 : id + 1);
    }

    return ret;
}

void SendUnitList(const vector<size_t>& command_list)
{
    nlohmann::json obj;
    obj["positions"] = command_list;
    cerr << obj.dump() << endl;
    cout << obj.dump() << endl;
}

Command SelectCommand(SparseGraph<Node>& graph, const std::size_t selfId)
{
    for (std::size_t i = 0; i < graph.size(); i++)
    {
        if (graph.node(i).PlayerId == selfId)
        {
            for (auto next : graph.neighbor(i))
            {
                if (graph.node(next).PlayerId == -1)
                {
                    return Command(i, next, graph.node(i).UnitCount);
                }
            }
        }
    }
    assert(false);
}

void SendCommandList(const Command& command)
{
    nlohmann::json obj;
    obj["src"] = command.SrcNodeId;
    obj["dst"] = command.DstNodeId;
    obj["num"] = command.UnitNum;
    cerr << obj.dump() << endl;
    cout << obj.dump() << endl;
}

int main()
{
    size_t playerId;

    while (true)
    {
        string line;
        // 1行で json が吐き出される
        getline(cin, line);
        nlohmann::json obj = nlohmann::json::parse(line);

        if (obj["action"] == "init")
        {
            auto graph = ParseGraph(obj);
            playerId = obj["uid"];
            const size_t UnitSize = obj["num_units"];
            SendUnitList(SelectUnitList(graph, UnitSize));
        }
        else if (obj["action"] == "play")
        {
            auto graph = ParseGraph(obj);
            SendCommandList(SelectCommand(graph, playerId));
        }
        else if (obj["action"] == "quit")
        {
            cout << endl;
            break;
        }
    }
}