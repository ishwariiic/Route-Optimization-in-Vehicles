#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <tuple>
#include <limits>
using namespace std;

#define INF numeric_limits<int>::max()

struct Intersection {
    int id;
    double x, y;
};

struct VehicleState {
    int id;    
    int speed; 
    int position; 
    VehicleState* next;
};


class LinkedList {
private:
    VehicleState* head;
public:
    LinkedList() : head(nullptr) {}

    void addState(int id, int speed, int position) {
        VehicleState* newState = new VehicleState{id, speed, position, head};
        head = newState;
    }

    void displayStates() {
        VehicleState* current = head;
        while (current) {
            cout << "Vehicle ID: " << current->id 
                 << ", Speed: " << current->speed 
                 << ", Position: " << current->position << endl;
            current = current->next;
        }
    }

    ~LinkedList() {
        // Destructor to free memory
        VehicleState* current = head;
        while (current) {
            VehicleState* temp = current;
            current = current->next;
            delete temp;
        }
    }
};

double calculate_heuristic(const Intersection& a, const Intersection& b) {
    return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

void dijkstra(int src, int V, vector<vector<pair<int, int>>>& adj) {
    vector<int> dist(V, INF);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    dist[src] = 0;
    pq.push({0, src});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        for (auto edge : adj[u]) {
            int v = edge.first;
            int weight = edge.second;

            if (dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
                pq.push({dist[v], v});
            }
        }
    }

    cout << "Dijkstra's shortest distances from source " << src << ":\n";
    for (int i = 0; i < V; i++) {
        cout << "Intersection " << i << " : " << dist[i] << "\n";
    }
}

void a_star(int src, int dest, int V, vector<vector<pair<int, int>>>& adj, vector<Intersection>& intersections) {
    vector<int> dist(V, INF);
    priority_queue<pair<int, int>, vector<pair<int, int>>, greater<pair<int, int>>> pq;

    dist[src] = 0;
    pq.push({0, src});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (u == dest) break;

        for (auto edge : adj[u]) {
            int v = edge.first;
            int weight = edge.second;

            double heuristic = calculate_heuristic(intersections[v], intersections[dest]);

            if (dist[u] + weight + heuristic < dist[v]) {
                dist[v] = dist[u] + weight + heuristic;
                pq.push({dist[v], v});
            }
        }
    }

    cout << "A* shortest path from source " << src << " to destination " << dest << " is: " << dist[dest] << "\n";
}

void bellman_ford(int src, int V, vector<vector<pair<int, int>>>& adjList) {
    vector<int> dist(V, INF);
    dist[src] = 0;

    vector<tuple<int, int, int>> edges;
    for (int u = 0; u < V; u++) {
        for (auto edge : adjList[u]) {
            edges.push_back({u, edge.first, edge.second});
        }
    }

    for (int i = 0; i < V - 1; i++) {
        for (auto [u, v, weight] : edges) {
            if (dist[u] != INF && dist[u] + weight < dist[v]) {
                dist[v] = dist[u] + weight;
            }
        }
    }

    for (auto [u, v, weight] : edges) {
        if (dist[u] != INF && dist[u] + weight < dist[v]) {
            cout << "Negative weight cycle (traffic loop or accident) detected!\n";
            return;
        }
    }

    cout << "Bellman-Ford shortest distances from source " << src << ":\n";
    for (int i = 0; i < V; i++) {
        cout << "Intersection " << i << " : " << dist[i] << "\n";
    }
}

void floyd_warshall(int V, vector<vector<int>>& graph) {
    vector<vector<int>> dist = graph;

    for (int k = 0; k < V; k++) {
        for (int i = 0; i < V; i++) {
            for (int j = 0; j < V; j++) {
                if (dist[i][k] != INF && dist[k][j] != INF && dist[i][k] + dist[k][j] < dist[i][j]) {
                    dist[i][j] = dist[i][k] + dist[k][j];
                }
            }
        }
    }

    cout << "Floyd-Warshall shortest distances matrix:\n";
    for (int i = 0; i < V; i++) {
        for (int j = 0; j < V; j++) {
            if (dist[i][j] == INF) {
                cout << "INF ";
            } else {
                cout << dist[i][j] << " ";
            }
        }
        cout << "\n";
    }
}


struct TreeNode {
    int vehicleId;
    int position;
    TreeNode* left;
    TreeNode* right;
};


TreeNode* insert(TreeNode* root, int vehicleId, int position) {
    if (root == nullptr) {
        return new TreeNode{vehicleId, position, nullptr, nullptr};
    }

    if (position < root->position) {
        root->left = insert(root->left, vehicleId, position);
    } else {
        root->right = insert(root->right, vehicleId, position);
    }
    return root;
}

void inorder(TreeNode* root) {
    if (root) {
        inorder(root->left);
        cout << "Vehicle ID: " << root->vehicleId << ", Position: " << root->position << endl;
        inorder(root->right);
    }
}

int main() {
    int V = 5;

    vector<Intersection> intersections = {
        {0, 0.0, 0.0},
        {1, 1.0, 1.0},
        {2, 2.0, 2.0},
        {3, 3.0, 3.0},
        {4, 4.0, 4.0}
    };

    vector<vector<pair<int, int>>> adj(V);


    adj[0].push_back({1, 10});
    adj[0].push_back({4, 5});
    adj[1].push_back({2, 1});
    adj[2].push_back({3, 4});
    adj[3].push_back({0, 7});
    adj[4].push_back({1, 3});
    adj[4].push_back({2, 9});

    vector<vector<int>> graph(V, vector<int>(V, INF));
    graph[0][1] = 10; graph[0][4] = 5;
    graph[1][2] = 1; graph[2][3] = 4;
    graph[3][0] = 7; graph[4][1] = 3; graph[4][2] = 9;

    int src = 0, dest = 3;

    
    LinkedList vehicleStates;
    vehicleStates.addState(101, 60, 100); 
    vehicleStates.addState(102, 70, 150);
    vehicleStates.addState(103, 55, 80);

    cout << "Vehicle States:\n";
    vehicleStates.displayStates();

    
    TreeNode* root = nullptr;
    root = insert(root, 101, 100);
    root = insert(root, 102, 150);
    root = insert(root, 103, 80);

    cout << "Vehicle States (BST In-Order):\n";
    inorder(root); 

    
    dijkstra(src, V, adj);
    a_star(src, dest, V, adj, intersections);
    bellman_ford(src, V, adj);
    floyd_warshall(V, graph);


    return 0;
}
