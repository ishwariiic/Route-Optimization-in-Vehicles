#include <iostream>
#include <vector>
#include <queue>
#include <cmath>
#include <tuple>
#include <limits>
using namespace std;

#define INF numeric_limits<double>::max()

struct Intersection {
    int id;
    double x, y;
    bool has_traffic;   
    double elevation;   
};


double calculate_heuristic(const Intersection& a, const Intersection& b) {
    double distance = sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
    return distance;
}


double calculate_dynamic_weight(int base_weight, const Intersection& u, const Intersection& v) {
    double traffic_factor = (v.has_traffic) ? 1.5 : 1.0;
    double elevation_diff = abs(v.elevation - u.elevation); 
    

    return base_weight * traffic_factor + 0.2 * elevation_diff;
}

void a_star(int src, int dest, int V, vector<vector<pair<int, int>>>& adj, vector<Intersection>& intersections) {
    vector<double> dist(V, INF);
    vector<int> parent(V, -1); 
    priority_queue<pair<double, int>, vector<pair<double, int>>, greater<pair<double, int>>> pq;

    dist[src] = 0;
    pq.push({0, src});

    while (!pq.empty()) {
        int u = pq.top().second;
        pq.pop();

        if (u == dest) break; 

        for (auto edge : adj[u]) {
            int v = edge.first;
            int base_weight = edge.second;

        
            double dynamic_weight = calculate_dynamic_weight(base_weight, intersections[u], intersections[v]);
            
           
            double heuristic = calculate_heuristic(intersections[v], intersections[dest]);
            double new_cost = dist[u] + dynamic_weight;

            if (new_cost + heuristic < dist[v]) {
                dist[v] = new_cost;
                parent[v] = u;
                pq.push({dist[v] + heuristic, v});
            }
        }
    }

 
    if (dist[dest] == INF) {
        cout << "No path found from " << src << " to " << dest << "\n";
        return;
    }

    cout << "Optimized A* path from source " << src << " to destination " << dest << " is: " << dist[dest] << "\n";
    
}

int main() {
    int V = 5;

    vector<Intersection> intersections = {
        {0, 0.0, 0.0, false, 100.0},
        {1, 1.0, 1.0, true, 150.0},
        {2, 2.0, 2.0, false, 130.0},
        {3, 3.0, 3.0, false, 110.0},
        {4, 4.0, 4.0, true, 180.0}
    };

    vector<vector<pair<int, int>>> adj(V);
    adj[0].push_back({1, 10});
    adj[0].push_back({4, 5});
    adj[1].push_back({2, 1});
    adj[2].push_back({3, 4});
    adj[3].push_back({0, 7});
    adj[4].push_back({1, 3});
    adj[4].push_back({2, 9});

    int src = 0, dest = 3;
    a_star(src, dest, V, adj, intersections);

    return 0;
}
