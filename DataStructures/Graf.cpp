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

    string line;
    while (getline(inputFile, line)) {
        istringstream iss(line);
        int id, longitude, latitude;
        if (!(iss >> id >> longitude >> latitude)) {
            cerr << "Error parsing line: " << line << endl;
            continue;
        }

        graph.addNode(id, longitude, latitude);
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

vector<vector<double>> Graph::createAdjacencyMatrix() {
    int numNodes = nodes.size();
    vector<vector<double>> adjacencyMatrix(numNodes, vector<double>(numNodes, 0.0));

    for (const auto& edge : edges) {
        adjacencyMatrix[edge.origin][edge.destination] = calculateDistance(edge.origin, edge.destination);
        adjacencyMatrix[edge.destination][edge.origin] = adjacencyMatrix[edge.origin][edge.destination];
    }

    return adjacencyMatrix;
}

std::vector<int> Graph::tspTriangularApproximation(vector<vector<double>> aux) {
    std::vector<int> tour;
    std::unordered_set<int> visited;
    tour.push_back(0);  // Start with node 0
    visited.insert(0);

    while (visited.size() < getNumNodes()) {
        int current = tour.back();
        double minDistance = std::numeric_limits<double>::max();
        int nearestNeighbor = -1;

        for (int neighbor = 0; neighbor < getNumNodes(); ++neighbor) {
            if (visited.count(neighbor) == 0 && aux[current][neighbor] < minDistance) {
                minDistance = aux[current][neighbor];
                nearestNeighbor = neighbor;
            }
        }

        if (nearestNeighbor != -1) {
            tour.push_back(nearestNeighbor);
            visited.insert(nearestNeighbor);
        }
    }

    tour.push_back(0);  // Complete the cycle by adding the starting node to the end
    return tour;
}