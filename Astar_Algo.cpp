#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <limits>
#include <unordered_map>
#include <string>
#include <algorithm>

using namespace std;

struct Node 
{
    int vertex;
    double weight;
    Node(int v, double w) : vertex(v), weight(w) {}
};

struct Compare 
{
    bool operator()(Node const& a, Node const& b) {
        return a.weight > b.weight;
    }
};

class Graph 
{
public:
    int V;
    vector<vector<Node>> adjList;
    unordered_map<int, pair<double, double>> coordinates;

    Graph(int vertices) : V(vertices), adjList(vertices) {}

    void addEdge(int u, int v, double weight) {
        adjList[u].push_back(Node(v, weight));
        adjList[v].push_back(Node(u, weight));
    }

    void setCoordinates(int vertex, double x, double y) 
    {
        coordinates[vertex] = make_pair(x, y);
    }

    vector<double> dijkstra(int start) {
        vector<double> dist(V, numeric_limits<double>::max());
        dist[start] = 0;
        priority_queue<Node, vector<Node>, Compare> pq;
        pq.push(Node(start, 0));

        while (!pq.empty()) {
            int u = pq.top().vertex;
            double uDist = pq.top().weight;
            pq.pop();

            if (uDist > dist[u]) continue;

            for (const auto& neighbor : adjList[u]) {
                int v = neighbor.vertex;
                double weight = neighbor.weight;
                if (dist[v] > uDist + weight) {
                    dist[v] = uDist + weight;
                    pq.push(Node(v, dist[v]));
                }
            }
        }
        return dist;
    }

    double heuristic(int u, int goal) {
        if (coordinates.find(u) == coordinates.end() || coordinates.find(goal) == coordinates.end()) {
            return abs(u - goal);
        }
        double x1 = coordinates[u].first;
        double y1 = coordinates[u].second;
        double x2 = coordinates[goal].first;
        double y2 = coordinates[goal].second;
        return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
    }

    vector<int> aStar(int start, int goal) {
        vector<double> dist(V, numeric_limits<double>::max());
        vector<int> parent(V, -1);
        dist[start] = 0;
        priority_queue<Node, vector<Node>, Compare> pq;
        pq.push(Node(start, 0));

        vector<double> fScore(V, numeric_limits<double>::max());
        fScore[start] = heuristic(start, goal);

        while (!pq.empty()) {
            int u = pq.top().vertex;
            pq.pop();

            if (u == goal) break;

            for (const auto& neighbor : adjList[u]) {
                int v = neighbor.vertex;
                double weight = neighbor.weight;
                double tentative_gScore = dist[u] + weight;

                if (tentative_gScore < dist[v]) {
                    parent[v] = u;
                    dist[v] = tentative_gScore;
                    fScore[v] = dist[v] + heuristic(v, goal);
                    pq.push(Node(v, fScore[v]));
                }
            }
        }

        vector<int> path;
        for (int v = goal; v != -1; v = parent[v]) {
            path.push_back(v);
        }
        reverse(path.begin(), path.end());
        return path;
    }

    void updateEdge(int u, int v, double newWeight) {
        for (auto& neighbor : adjList[u]) {
            if (neighbor.vertex == v) {
                neighbor.weight = newWeight;
                break;
            }
        }
        for (auto& neighbor : adjList[v]) {
            if (neighbor.vertex == u) {
                neighbor.weight = newWeight;
                break;
            }
        }
    }

    void removeEdge(int u, int v) {
        adjList[u].erase(remove_if(adjList[u].begin(), adjList[u].end(),
                                   [v](const Node& n) { return n.vertex == v; }),
                         adjList[u].end());
        adjList[v].erase(remove_if(adjList[v].begin(), adjList[v].end(),
                                   [u](const Node& n) { return n.vertex == u; }),
                         adjList[v].end());
    }
};
