#include <algorithm>
#include <chrono>
#include <cstdint>
#include <iostream>
#include <map>
#include <queue>
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

constexpr int EMPTY_PLAYER_ID = -1;

struct Node
{
    int PlayerId;
    int UnitCount;
    // bfs する時の距離計算用 tmp
    int distance;
    // 復元用のグラフ
    int prev_node;

    Node()
        : PlayerId(EMPTY_PLAYER_ID)
        , UnitCount(0)
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

    size_t accumulate_time;

    size_t DFS(const SparseGraph<Node>& graph, const std::size_t node, vector<bool>& visited);

    void BFS(SparseGraph<Node>& graph, const std::size_t node);

    size_t VisitableNodeSize(const SparseGraph<Node>& graph, const std::size_t base, const std::size_t next);

    vector<pair<size_t, size_t>> DivisionPositionList(const SparseGraph<Node>& graph);

    chrono::system_clock::time_point start_;

    void StartTimer();

    void StopTimer();

    bool TimeLimitExceeded()
    {
        cerr << accumulate_time << " | " << (5000 * 1000) << endl;
        return accumulate_time > 5000 * 1000;
    }

    size_t uid_;
};

NodelessSolver::NodelessSolver()
{
    accumulate_time = 0;
}

void NodelessSolver::StartTimer()
{
    start_ = chrono::system_clock::now();
}

void NodelessSolver::StopTimer()
{
    auto end = chrono::system_clock::now();
    accumulate_time += chrono::duration_cast<chrono::microseconds>(end - start_).count();
}

vector<size_t> NodelessSolver::SelectUnitList(SparseGraph<Node>& graph, const size_t unit_size, mt19937_64& mt)
{
    StartTimer();

    vector<double> score_list(graph.size(), 0);
    vector<size_t> candidate;
    for (size_t i = 0; i < graph.size(); i++)
    {
        candidate.push_back(i);
    }
    sort(candidate.begin(), candidate.end(), [&](const size_t i, const size_t j) {
        return graph.neighbor(i).size() < graph.neighbor(j).size();
    });

    vector<size_t> ret;
    vector<bool> selected(graph.size(), false);
    for (auto node : candidate)
    {
        for (auto next : graph.neighbor(node))
        {
            if (!selected[next])
            {
                selected[next] = true;
                ret.push_back(next);
            }
            if (ret.size() == unit_size)
            {
                break;
            }
        }
        if (ret.size() == unit_size)
        {
            break;
        }
    }

    StopTimer();

    return ret;
}

// SelectCommand

void NodelessSolver::BFS(SparseGraph<Node>& graph, const std::size_t init)
{
    queue<int> que;
    que.emplace(init);
    for (size_t i = 0; i < graph.size(); i++)
    {
        graph.node(i).distance = graph.size() + 1;
    }

    graph.node(init).distance = 0;
    while (!que.empty())
    {
        auto node = que.front();
        que.pop();
        const auto prev_dist = graph.node(node).distance;

        for (auto next : graph.neighbor(node))
        {
            auto& nn = graph.node(next);
            if ((nn.PlayerId == EMPTY_PLAYER_ID || (nn.PlayerId == UID() && nn.UnitCount > 0)) && nn.distance > prev_dist + 1)
            {
                nn.distance = prev_dist + 1;
                nn.prev_node = node;
                que.push(next);
            }
        }
    }
}

size_t NodelessSolver::DFS(const SparseGraph<Node>& graph, const std::size_t node, vector<bool>& visited)
{
    visited[node] = true;
    size_t ret = 1;
    for (auto next : graph.neighbor(node))
    {
        if (!visited[next] && graph.node(next).PlayerId == EMPTY_PLAYER_ID)
        {
            ret += DFS(graph, next, visited);
        }
    }
    return ret;
}

size_t NodelessSolver::VisitableNodeSize(const SparseGraph<Node>& graph, const size_t base, const size_t next)
{
    vector<bool> visited(graph.size(), false);
    visited[base] = true;
    return DFS(graph, next, visited);
}

vector<pair<size_t, size_t>> NodelessSolver::DivisionPositionList(const SparseGraph<Node>& graph)
{
    vector<pair<size_t, size_t>> result;

    for (size_t i = 0; i < graph.size(); i++)
    {
        if (graph.node(i).PlayerId == EMPTY_PLAYER_ID)
        {
            vector<bool> visited(graph.size(), false);
            const size_t all_empty_count = DFS(graph, i, visited);

            std::size_t min_cluster_size = all_empty_count;

            // 隣接から DFS して訪れた場所は保存しておく
            fill(visited.begin(), visited.end(), false);
            visited[i] = true;

            std::size_t rest_cluster_size = all_empty_count - 1;

            for (auto next : graph.neighbor(i))
            {
                if (!visited[next] && graph.node(next).PlayerId == EMPTY_PLAYER_ID)
                {
                    const auto empty_size = DFS(graph, next, visited);
                    if (0 < empty_size && empty_size < all_empty_count - 1)
                    {
                        min_cluster_size = min(min_cluster_size, empty_size);
                    }
                    rest_cluster_size -= empty_size;
                    if (rest_cluster_size == 0)
                    {
                        break;
                    }
                }
            }
            if (min_cluster_size < all_empty_count)
            {
                result.emplace_back(min_cluster_size, i);
            }
        }
    }
    return result;
}

Command NodelessSolver::SelectCommand(SparseGraph<Node>& graph, mt19937_64& mt)
{
    StartTimer();

    if (!TimeLimitExceeded())
    {
        auto division_list = DivisionPositionList(graph);

        if (!division_list.empty())
        {
            sort(division_list.begin(), division_list.end(), greater<>());
            const auto [_, node] = division_list.front();

            BFS(graph, node);

            int min_dist = graph.size() + 1;
            int min_unit = graph.size() + 1;
            for (int i = 0; i < graph.size(); i++)
            {
                const auto& nn = graph.node(i);
                if (nn.PlayerId == UID() && nn.UnitCount > 0)
                {
                    if (nn.distance < min_dist)
                    {
                        min_dist = nn.distance;
                        min_unit = i;
                    }
                }
            }
            if (min_unit != graph.size() + 1)
            {
                StopTimer();
                return Command(min_unit, graph.node(min_unit).prev_node, 1);
            }
        }
    }

    vector<size_t> candidate;
    for (size_t i = 0; i < graph.size(); i++)
    {
        candidate.push_back(i);
    }
    shuffle(candidate.begin(), candidate.end(), mt);

    for (auto i : candidate)
    {
        const auto& node = graph.node(i);
        if (node.PlayerId == UID() && node.UnitCount > 0)
        {
            vector<size_t> next_list;
            for (auto next : graph.neighbor(i))
            {
                if (graph.node(next).PlayerId == EMPTY_PLAYER_ID)
                {
                    next_list.push_back(next);
                }
            }
            if (!next_list.empty())
            {
                int best_eval = -100;
                size_t best_node = graph.size();
                for (auto next : next_list)
                {
                    int eval = VisitableNodeSize(graph, i, next);
                    if (best_eval < eval)
                    {
                        best_eval = eval;
                        best_node = next;
                    }
                }
                assert(graph.size() != best_node);
                StopTimer();
                return Command(i, best_node, 1);
            }
        }
    }
    assert(false);
}

// -- end implementation --

SparseGraph<Node> ParseGraph(const nlohmann::json& obj)
{
    SparseGraph<Node> graph;
    const auto& node_list = obj["nodes"];
    const auto& edge_list = obj["edges"];

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

    const auto& states = obj["state"];
    for (size_t i = 0; i < states.size(); i++)
    {
        graph.node(i).PlayerId = states[i];
    }

    const auto& units = obj["units"];
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