#include "Graf.h"
#include "math.h"
//#include "ctime"
#include <chrono>
#include <iostream>
#include <vector>
#include <algorithm>
#include <cmath>
#include <limits>

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
            if (i == ii) dists[i][ii] = 0;
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


    inputFile.close();


    auto **ptr = new const float*[n];
    for (unsigned int i = 0; i < n; i++)
        ptr[i] = dists[i];
    unsigned int path[n];


    auto start = std::chrono::high_resolution_clock::now();

    float answer = tspBT(ptr, n, path);

    auto end = std::chrono::high_resolution_clock::now();

    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();





    for (int i = 0; i < n; i++){
        cout << path[i] << " - ";
    }

    cout << "0" << endl;

    cout << "Distance: ";
    cout << answer << endl;

    cout << "It took " << duration << " milliseconds to complete the algoritm." << endl;

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


//tentative christofers



const double INF = std::numeric_limits<double>::max();

// Function to find the minimum element from an array
int Graph::findMin(std::vector<double>& dist, std::vector<bool>& visited) {
    double minDist = INF;
    int minIndex = -1;
    int n = dist.size();

    for (int i = 0; i < n; ++i) {
        if (!visited[i] && dist[i] < minDist) {
            minDist = dist[i];
            minIndex = i;
        }
    }

    return minIndex;
}

// Function to find the minimum spanning tree using Prim's algorithm
std::vector<std::vector<double>> Graph::minimumSpanningTree(const std::vector<std::vector<double>>& adjMatrix) {
    int n = adjMatrix.size();

    std::vector<double> dist(n, INF);
    std::vector<int> parent(n, -1);
    std::vector<bool> visited(n, false);

    dist[0] = 0;

    for (int i = 0; i < n - 1; ++i) {
        int u = findMin(dist, visited);
        visited[u] = true;

        for (int v = 0; v < n; ++v) {
            if (!visited[v] && adjMatrix[u][v] < dist[v]) {
                dist[v] = adjMatrix[u][v];
                parent[v] = u;
            }
        }
    }

    std::vector<std::vector<double>> mst(n, std::vector<double>(n, 0.0));

    for (int i = 1; i < n; ++i) {
        mst[i][parent[i]] = mst[parent[i]][i] = adjMatrix[i][parent[i]];
    }

    return mst;
}

// Function to find the Eulerian circuit in a graph
void Graph::eulerianCircuit(int u, std::vector<std::vector<double>>& graph, std::vector<int>& circuit) {
    for (int v = 0; v < graph.size(); ++v) {
        if (graph[u][v] > 0) {
            graph[u][v] = graph[v][u] = 0;
            eulerianCircuit(v, graph, circuit);
        }
    }

    circuit.push_back(u);
}

// Function to find the minimum-weight perfect matching on a complete graph
std::vector<std::pair<int, int>> Graph::minimumWeightMatching(const std::vector<std::vector<double>>& adjMatrix) {
    int n = adjMatrix.size();
    std::vector<std::pair<int, int>> matching;

    std::vector<bool> matched(n, false);
    std::vector<double> minWeight(n, INF);

    for (int u = 0; u < n - 1; ++u) {
        if (!matched[u]) {
            int v = -1;

            for (int i = u + 1; i < n; ++i) {
                if (!matched[i] && adjMatrix[u][i] < minWeight[u]) {
                    minWeight[u] = adjMatrix[u][i];
                    v = i;
                }
            }

            if (v != -1) {
                matching.push_back(std::make_pair(u, v));
                matched[u] = matched[v] = true;
            }
        }
    }

    return matching;
}

// Function to shortcut the Eulerian circuit to form a Hamiltonian circuit
std::vector<int> Graph::shortcutCircuit(const std::vector<int>& eulerianCircuit) {
    std::vector<int> circuit;

    for (int vertex : eulerianCircuit) {
        if (std::find(circuit.begin(), circuit.end(), vertex) == circuit.end()) {
            circuit.push_back(vertex);
        }
    }

    return circuit;
}

// Function to compute the total weight of a given tour
double Graph::computeTourWeight(const std::vector<int>& tour, const std::vector<std::vector<double>>& adjMatrix) {
    double weight = 0.0;
    int n = tour.size();

    for (int i = 0; i < n - 1; ++i) {
        weight += adjMatrix[tour[i]][tour[i + 1]];
    }

    return weight;
}

// Function to apply the Christofides heuristic to the TSP problem
std::vector<int> Graph::christofides(const std::vector<std::vector<double>>& adjMatrix) {


    int n = adjMatrix.size();

    // Step 1: Create a minimum spanning tree (MST) from the adjacency matrix
    std::vector<std::vector<double>> mst = minimumSpanningTree(adjMatrix);

    // Step 2: Find vertices with odd degrees in the MST
    std::vector<int> oddVertices;
    for (int i = 0; i < n; ++i) {
        int degree = 0;
        for (int j = 0; j < n; ++j) {
            if (mst[i][j] > 0) {
                ++degree;
            }
        }
        if (degree % 2 != 0) {
            oddVertices.push_back(i);
        }
    }

    // Step 3: Create a complete graph from the odd vertices
    std::vector<std::vector<double>> completeGraph(n, std::vector<double>(n, 0.0));
    for (int i = 0; i < oddVertices.size(); ++i) {
        for (int j = i + 1; j < oddVertices.size(); ++j) {
            int u = oddVertices[i];
            int v = oddVertices[j];
            completeGraph[u][v] = completeGraph[v][u] = adjMatrix[u][v];
        }
    }

    // Step 4: Find minimum-weight perfect matching on the complete graph
    std::vector<std::pair<int, int>> matching = minimumWeightMatching(completeGraph);

    // Step 5: Combine the minimum spanning tree and the matching edges
    std::vector<std::vector<double>> combinedGraph = mst;
    for (const auto& edge : matching) {
        int u = edge.first;
        int v = edge.second;
        combinedGraph[u][v] = combinedGraph[v][u] = adjMatrix[u][v];
    }

    // Step 6: Find an Eulerian circuit in the combined graph
    std::vector<int> eulerianCircuit_;
    eulerianCircuit(0, combinedGraph, eulerianCircuit_);

    // Step 7: Create a Hamiltonian circuit by shortcutting the Eulerian circuit
    std::vector<int> hamiltonianCircuit = shortcutCircuit(eulerianCircuit_);

    // Rotate the circuit to start and end at vertex 0
    auto it = std::find(hamiltonianCircuit.begin(), hamiltonianCircuit.end(), 0);
    std::rotate(hamiltonianCircuit.begin(), it, hamiltonianCircuit.end());

    // Ensure the circuit ends at vertex 0
    hamiltonianCircuit.push_back(0);



    return hamiltonianCircuit;
}

// Function to print a tour
void Graph::printTour(const std::vector<int>& tour, const std::vector<vector<double>> &v) {
    double dist = 0;
    for (int vertex : tour) {
        std::cout << vertex << " ";
    }
    std::cout << std::endl;
    for (int i = 1; i < tour.size(); i++){
        dist += v[tour[i-1]][tour[i]];
    }
    std::cout << "Distance: " << dist;
    std::cout << std::endl;
}

void Graph::populateMatrix(string filename, vector<vector<double>> &v){
    int n = countNodes(filename);

    for (int i = 0; i < n; i++){
        vector<double> vv;
        for (int ii = 0; ii < n; ii++){
            if (i == ii) vv.push_back(0);
            else vv.push_back(INF);
        }
        v.push_back(vv);
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
        if (filename == "../Data/Toy-Graphs/tourism.csv"|| filename == "../Data/Toy-Graphs/stadiums.csv"|| filename =="../Data/Toy-Graphs/shipping.csv")
        getline(inputFile, line); //livrar a primeira linha que é só as designaçoes


        while (getline(inputFile, line)) {

            stringstream inputString(line);

            getline(inputString, nodeA, ',');
            getline(inputString, nodeB, ',');
            getline(inputString, dist, ',');

            distance = atof(dist.c_str());
            nA = atoi(nodeA.c_str());
            nB = atoi(nodeB.c_str());


            v[nA][nB] = distance;
            v[nB][nA] = distance;
        }

        inputFile.close();
}
