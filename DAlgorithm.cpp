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
