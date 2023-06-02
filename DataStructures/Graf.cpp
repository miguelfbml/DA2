#include "Graf.h"
#include "math.h"
//#include "ctime"
#include <chrono>

void Graph::populateGraph_nodes(Graph& graph, const string& filename) {
    ifstream inputFile(filename);
    if (!inputFile) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }
    string nodeA;
    string nodeB;
    string dist;
    float distance;
    int nA;
    float nB;

    string line;
    getline(inputFile, line); //livrar a primeira linha que é só as designaçoes


    while (getline(inputFile, line)) {

        stringstream inputString(line);

        getline(inputString, nodeA, ',');
        getline(inputString, nodeB, ',');
        getline(inputString, dist, ',');

        distance = atof(dist.c_str());
        nA = atoi(nodeA.c_str());
        nB = atof(nodeB.c_str());


        graph.addNode(nA, nB, distance);
    }
    inputFile.close();
}

int Graph::countNodes(string& filename){
    unordered_set<int> aux_nodes;
    ifstream inputFile(filename);
    if (!inputFile) {
        cerr << "Error opening file: " << filename << endl;
        return -1;
    }

    string nodeA;
    string nodeB;
    string dist;
    int nA;
    int nB;

    string line;
    getline(inputFile, line);

    while (getline(inputFile, line)) {

        stringstream inputString(line);

        getline(inputString, nodeA, ',');
        getline(inputString, nodeB, ',');
        getline(inputString, dist, ',');

        nA = atoi(nodeA.c_str());
        nB = atoi(nodeB.c_str());

        aux_nodes.insert(nA);
        aux_nodes.insert(nB);
    }

    return aux_nodes.size();

}

void Graph::populateToyNodes(const string& filename, int n){

    float dists[n][n];

    for (int i = 0; i < n; i++){
        for (int ii = 0; ii < n; ii++){
            dists[i][ii] = 50000000;
        }
    }



    ifstream inputFile(filename);
    if (!inputFile) {
        cerr << "Error opening file: " << filename << endl;
    }
    string nodeA;
    string nodeB;
    string dist;
    float distance;
    int nA;
    int nB;


    string line;
    getline(inputFile, line); //livrar a primeira linha que é só as designaçoes


    while (getline(inputFile, line)) {

        stringstream inputString(line);

        getline(inputString, nodeA, ',');
        getline(inputString, nodeB, ',');
        getline(inputString, dist, ',');

        distance = atof(dist.c_str());
        nA = atoi(nodeA.c_str());
        nB = atoi(nodeB.c_str());


        dists[nA][nB] = distance;
        dists[nB][nA] = distance;
    }

    for (int i = 0; i < n; i++){
        for (int  ii = 0; ii < n; ii++){
            cout << dists[i][ii] << " - ";
        }
        cout << "( " << i << " )";
        cout << endl;
    }

    inputFile.close();


    auto **ptr = new const float*[n];
    for (unsigned int i = 0; i < n; i++)
        ptr[i] = dists[i];
    unsigned int path[n];


    auto start = std::chrono::high_resolution_clock::now();

    float answer = tspBT(ptr, n, path);

    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();





    cout << answer << endl;
    cout << "It took " << duration << " milliseconds to complete the algoritm." << endl;

    for (int i = 0; i < n; i++){
        cout << path[i] << " - ";
    }

    cout << "0" << endl;


}

void Graph::populateGraph_edges(Graph& graph, const string& filename) {
    ifstream inputFile(filename);
    if (!inputFile) {
        cerr << "Error opening file: " << filename << endl;
        return;
    }
    string nodeA;
    string nodeB;
    string dist;
    float distance;
    int nA;
    int nB;

    string line;
    getline(inputFile, line); //livrar a primeira linha que é só as designaçoes


    while (getline(inputFile, line)) {

        stringstream inputString(line);

        getline(inputString, nodeA, ',');
        getline(inputString, nodeB, ',');
        getline(inputString, dist, ',');

        distance = atof(dist.c_str());
        nA = atoi(nodeA.c_str());
        nB = atoi(nodeB.c_str());

        graph.addEdge(nA, nB, distance);
    }
     inputFile.close();
}

void Graph::addNode(int nodeId, double longitude, double latitude) {
    Node node;
    node.id = nodeId;
    node.longitude = longitude;
    node.latitude = latitude;
    nodes[nodeId] = node;
}

void Graph::addEdge(int origin, int destination, double distance) {
    edges.push_back({origin, destination, distance});
}

double Graph::calculateDistance(int origin, int destination) {
    Node node1 = nodes[origin];
    Node node2 = nodes[destination];
    return haversineDistance(node1.latitude, node1.longitude, node2.latitude, node2.longitude);
}

int Graph::getNumNodes() const {
    return nodes.size();
}

int Graph::getNumEdges() const {
    return edges.size();
}

vector<int> Graph::getNeighbors(int nodeId) {
    vector<int> neighbors;
    for (const auto& edge : edges) {
        if (edge.origin == nodeId) {
            neighbors.push_back(edge.destination);
        }
    }
    return neighbors;
}

vector<int> Graph::getNodes() {
    vector<int> allNodes;
    for (const auto& node : nodes) {
        allNodes.push_back(node.first);
    }
    return allNodes;
}

double Graph::haversineDistance(double lat1, double lon1, double lat2, double lon2) {
    constexpr double earthRadiusKm = 6371.0;
    double dLat = degreesToRadians(lat2 - lat1);
    double dLon = degreesToRadians(lon2 - lon1);
    double a = sin(dLat / 2) * sin(dLat / 2) +
                cos(degreesToRadians(lat1)) * cos(degreesToRadians(lat2)) *
                sin(dLon / 2) * sin(dLon / 2);
    double c = 2 * atan2(sqrt(a), sqrt(1 - a));
    return earthRadiusKm * c;
}

double Graph::degreesToRadians(double degrees) {
    return degrees * (M_PI / 180.0);
}

vector<vector<double>> Graph::createAdjacencyMatrix(bool flag, int size) {
    int numNodes;
    if(size == 0) numNodes = nodes.size();
    else numNodes = size;
    double maxnum = numeric_limits<double>::max();
    vector<vector<double>> adjacencyMatrix(numNodes, vector<double>(numNodes, maxnum));


    for (int i = 0; i < numNodes; i++){
        adjacencyMatrix[i][i] = 0.0;
    }

    if(flag) {
        for (const auto &edge: edges) {
            adjacencyMatrix[edge.origin][edge.destination] = edge.distance;
            adjacencyMatrix[edge.destination][edge.origin] = adjacencyMatrix[edge.origin][edge.destination];
        }
    }

    else {
        for (const auto &node: nodes) {
            for (const auto &node1: nodes) {
                if (node.first == node1.first) {
                    continue;
                }
                adjacencyMatrix[node.first][node1.first] = calculateDistance(node.first, node1.first);

                adjacencyMatrix[node1.first][node.first] = adjacencyMatrix[node.first][node1.first];
            }
        }
    }


    return adjacencyMatrix;
}

std::vector<int> Graph::tspTriangularApproximation(vector<vector<double>> aux, double &distfinal, int size) {
    std::vector<int> tour;
    std::unordered_set<int> visited;
    tour.push_back(0);  // Start with node 0
    visited.insert(0);

    int number;
    if(size != 0) number = size;
    else number = getNumNodes();

    while (visited.size() < number) {
        int current = tour.back();
        double minDistance = std::numeric_limits<double>::max();
        int nearestNeighbor = -1;

        for (int neighbor = 0; neighbor < number; ++neighbor) {
            if (visited.count(neighbor) == 0 && aux[current][neighbor] < minDistance) {
                minDistance = aux[current][neighbor];
                nearestNeighbor = neighbor;
            }
        }

        if (nearestNeighbor != -1) {
            distfinal += minDistance;
            tour.push_back(nearestNeighbor);
            visited.insert(nearestNeighbor);
        }
    }
    distfinal += aux[tour.back()][0];
    tour.push_back(0);  // Complete the cycle by adding the starting node to the end
    return tour;
}

std::vector<int> Graph::findMinimumWeightPerfectMatching(Graph& graph, const std::vector<int>& oddDegreeNodes) {
    std::vector<int> matching;

    // Create a set to track matched nodes
    std::unordered_set<int> matchedNodes;

    // Iterate over each odd-degree node and find the minimum-weight edge
    // connecting it to an unmatched node
    for (int node : oddDegreeNodes) {
        double minWeight = std::numeric_limits<double>::max();
        int minNeighbor = -1;

        for (int neighbor : graph.getNeighbors(node)) {
            if (matchedNodes.find(neighbor) == matchedNodes.end()) {
                double weight = graph.calculateDistance(node, neighbor);
                if (weight < minWeight) {
                    minWeight = weight;
                    minNeighbor = neighbor;
                }
            }
        }

        if (minNeighbor != -1) {
            matching.push_back(node);
            matching.push_back(minNeighbor);
            matchedNodes.insert(node);
            matchedNodes.insert(minNeighbor);
        }
    }

    return matching;
}

std::vector<int> Graph::createEulerianCircuit(Graph& graph) {
    std::vector<int> eulerianCircuit;

    // Create a copy of the graph's adjacency matrix
    std::vector<std::vector<double>> adjacencyMatrix = graph.createAdjacencyMatrix(true);

    // Start with an arbitrary node and perform a depth-first search (DFS)
    // to construct the Eulerian circuit
    int startNode = graph.getNodes()[0];
    dfsEulerianCircuit(adjacencyMatrix, startNode, eulerianCircuit);

    return eulerianCircuit;
}

void Graph::dfsEulerianCircuit(std::vector<std::vector<double>>& adjacencyMatrix, int node, std::vector<int>& eulerianCircuit) {
    for (int neighbor = 0; neighbor < adjacencyMatrix.size(); ++neighbor) {
        while (adjacencyMatrix[node][neighbor] > 0) {
            adjacencyMatrix[node][neighbor]--;
            adjacencyMatrix[neighbor][node]--;
            dfsEulerianCircuit(adjacencyMatrix, neighbor, eulerianCircuit);
        }
    }
    eulerianCircuit.push_back(node);
}

void Graph::createMinimumSpanningTree(Graph& graph, Graph& mst) {
    std::vector<bool> visited(graph.getNumNodes() , false);  // Track visited nodes
    std::vector<int> parent(graph.getNumNodes(), -1);  // Store the parent of each node in the MST
    std::vector<double> key(graph.getNumNodes(), std::numeric_limits<double>::max());  // Key values used to pick the minimum weight edge

    // Start with an arbitrary node
    int startNode = 0;
    key[startNode] = 0.0;

    for (int i = 0; i < graph.getNumNodes()  - 1; ++i) {
        int u = getMinimumKeyIndex(graph, visited, key);
        visited[u] = true;

        for (int v : graph.getNeighbors(u)) {
            double weight = graph.calculateDistance(u, v);
            if (!visited[v] && weight < key[v]) {
                parent[v] = u;
                key[v] = weight;
            }
        }
    }

    // Add edges to the MST
    for (int i = 0; i < graph.getNumNodes(); ++i) {
        if (parent[i] != -1) {
            mst.addNode(i, nodes[i].longitude, nodes[i].latitude);
            mst.addEdge(parent[i], i, graph.calculateDistance(parent[i], i));
        }
    }
}

int Graph::getMinimumKeyIndex(const Graph& graph, const std::vector<bool>& visited, const std::vector<double>& key) {
    double minKey = std::numeric_limits<double>::max();
    int minIndex = -1;

    for (int i = 0; i < graph.getNumNodes(); ++i) {
        if (!visited[i] && key[i] < minKey) {
            minKey = key[i];
            minIndex = i;
        }
    }

    return minIndex;
}

void Graph::combineEdges(Graph& mst, const std::vector<int>& matching) {
    for (int i = 0; i < matching.size(); i += 2) {
        int origin = matching[i];
        int destination = matching[i + 1];
        double distance = mst.calculateDistance(origin, destination);
        mst.addEdge(origin, destination, distance);
    }
}

std::vector<int> Graph::tspHeuristic(Graph& graph) {
    std::vector<int> tspPath;

    // Step 1: Create a minimum spanning tree T of G
    Graph mst;
    createMinimumSpanningTree(graph, mst);

    // Step 2: Identify the set O of vertices with odd degrees in T
    std::vector<int> oddDegreeNodes;
    for (int nodeId : mst.getNodes()) {
        if (mst.getNeighbors(nodeId).size() % 2 != 0) {
            oddDegreeNodes.push_back(nodeId);
        }
    }

    // Step 3: Find a minimum-weight perfect matching M in the induced subgraph given by the vertices from O
    std::vector<int> matching = findMinimumWeightPerfectMatching(graph, oddDegreeNodes);

    // Step 4: Combine the edges of M and T to form a connected multigraph H in which each vertex has even degree
    combineEdges(mst, matching);

    // Step 5: Form an Eulerian circuit in H
    std::vector<int> eulerianCircuit = createEulerianCircuit(mst);

    // Step 6: Make the circuit found in the previous step into a Hamiltonian circuit by skipping repeated vertices
    std::unordered_set<int> visited;
    for (int nodeId : eulerianCircuit) {
        if (visited.find(nodeId) == visited.end()) {
            tspPath.push_back(nodeId);
            visited.insert(nodeId);
        }
    }

    return tspPath;
}