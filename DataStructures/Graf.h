#include <iostream>
#include <vector>
#include <unordered_set>
#include <unordered_map>
#include <limits>
#include <fstream>
#include <sstream>
#include <string>
#include "../src/tsp.h"


using namespace std;

#ifndef DA_TP_CLASSES_GRAF
#define DA_TP_CLASSES_GRAF

struct Node {
    int id;
    double longitude;
    double latitude;
};

class Graph {
public:

    static void populateGraph_nodes(Graph& graph, const string& filename);

    static void populateGraph_edges(Graph& graph, const string& filename);

    static void populateToyNodes(const string& filename, int n);

    void addNode(int nodeId, double longitude, double latitude);

    void addEdge(int origin, int destination, double distance);

    double calculateDistance(int origin, int destination);

    int getNumNodes() const;

    vector<int> getNeighbors(int nodeId);

    vector<int> getNodes();

    static int countNodes(string& filename);

    std::vector<int> tspTriangularApproximation(vector<vector<double>> aux);

    vector<vector<double>> createAdjacencyMatrix();


private:
    struct Edge {
        int origin;
        int destination;
        double distance;
    };

    unordered_map<int, Node> nodes;
    vector<Edge> edges;

    double haversineDistance(double lat1, double lon1, double lat2, double lon2);

    double degreesToRadians(double degrees);
};

#endif //DA_TP_CLASSES_GRAF