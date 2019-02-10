#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <map>
#include <random>
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

// -- start implementation --
// SelectUnitList

class NodelessSolver
{
public:
    NodelessSolver();

    vector<size_t> SelectUnitList(SparseGraph<Node>& graph, const size_t unit_size, mt19937_64& mt);

    Command SelectCommand(SparseGraph<Node>& graph, mt19937_64& mt);

    size_t UID() { return uid_; }

    void SetUID(const size_t uid) { this->uid_ = uid; }

private:
    const static int InitializationTimeLimit = 1000;

    const static int InitializationTimeCheckFrequency = 128;

    chrono::system_clock::time_point start;

    size_t uid_;
};

NodelessSolver::NodelessSolver()
{
    start = chrono::system_clock::now();
}

vector<size_t> NodelessSolver::SelectUnitList(SparseGraph<Node>& graph, const size_t unit_size, mt19937_64& mt)
{
    vector<double> score_list(graph.size(), 0);

    auto PushScore = [&](const int n1) {
        for (auto n2 : graph.neighbor(n1))
        {
            score_list[n2] += 1.0 / graph.neighbor(n1).size();
        }
    };

    auto RemoveScore = [&](const int n1) {
        for (auto n2 : graph.neighbor(n1))
        {
            score_list[n2] -= 1.0 / graph.neighbor(n1).size();
        }
    };

    auto CalculateScore = [&]() {
        double ret = 0;
        for (auto& s : score_list)
        {
            ret += sqrt(max(0.0, s));
        }
        return ret;
    };

    vector<size_t> ret;
    uniform_int_distribution<int> rand(0, graph.size() - 1);

    // initialize
    for (int i = 0; i < unit_size; i++)
    {
        const int n1 = rand(mt);
        ret.push_back(n1);
        PushScore(n1);
    }

    auto score = CalculateScore();
    double best_score = score;
    vector<size_t> best_ret(ret);

    double timerate = 0.0;

    uniform_real_distribution<double> rand_sa;

    auto Accept = [&](const double score_diff) {
        return score_diff >= 0.0 || rand_sa(mt) < exp(score_diff * timerate * 10);
    };

    uniform_int_distribution<int> rand_pos(0, unit_size - 1);
    int iter = 0;
    // SA
    while (true)
    {
        const auto index = rand_pos(mt);
        const auto prev_node = ret[index];
        const auto new_node = rand(mt);
        RemoveScore(prev_node);
        PushScore(new_node);
        const auto new_score = CalculateScore();
        if (Accept(new_score - score))
        {
            score = new_score;
            ret[index] = new_node;

            if (best_score < score)
            {
                best_score = score;
                copy(ret.begin(), ret.end(), best_ret.begin());
            }
        }
        else
        {
            RemoveScore(new_node);
            PushScore(prev_node);
        }

        if (iter++ % InitializationTimeCheckFrequency == 0)
        {
            auto end = chrono::system_clock::now();
            auto time = chrono::duration_cast<chrono::milliseconds>(end - start).count();
            timerate = static_cast<double>(time) / InitializationTimeLimit;
            if (time > InitializationTimeLimit)
            {
                break;
            }
        }
    }

    return best_ret;
}

// SelectCommand

Command NodelessSolver::SelectCommand(SparseGraph<Node>& graph, mt19937_64& mt)
{
    for (std::size_t i = 0; i < graph.size(); i++)
    {
        const auto& node = graph.node(i);
        if (node.PlayerId == UID() && node.UnitCount > 0)
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

// -- end implementation --

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
    }

    auto units = obj["units"];
    assert(units.size() == graph.size());
    for (std::size_t i = 0; i < units.size(); i++)
    {
        graph.node(i).UnitCount = units[i];
    }

    return graph;
}

void SendUnitList(const vector<size_t>& command_list)
{
    nlohmann::json obj;
    obj["positions"] = command_list;
    cout << obj.dump() << endl;
}

void SendCommandList(const Command& command)
{
    nlohmann::json obj;
    obj["src"] = command.SrcNodeId;
    obj["dst"] = command.DstNodeId;
    obj["num"] = command.UnitNum;
    cout << obj.dump() << endl;
}

int main()
{
    NodelessSolver solver;

    mt19937_64 mt;

    while (true)
    {
        string line;
        // 1行で json が吐き出される
        getline(cin, line);
        nlohmann::json obj = nlohmann::json::parse(line);

        if (obj["action"] == "init")
        {
            auto graph = ParseGraph(obj);
            solver.SetUID(obj["uid"]);
            const size_t UnitSize = obj["num_units"];
            SendUnitList(solver.SelectUnitList(graph, UnitSize, mt));
        }
        else if (obj["action"] == "play")
        {
            auto graph = ParseGraph(obj);
            SendCommandList(solver.SelectCommand(graph, mt));
        }
        else if (obj["action"] == "quit")
        {
            cout << endl;
            break;
        }
    }
}