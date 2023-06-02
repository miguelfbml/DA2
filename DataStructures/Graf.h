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

    std::vector<int> tspTriangularApproximation(vector<vector<double>> aux, double &distfinal, int size = 0);

    vector<vector<double>> createAdjacencyMatrix(bool flag = false, int size = 0);

    int getNumEdges() const;

    std::vector<int> findMinimumWeightPerfectMatching(Graph& graph, const std::vector<int>& oddDegreeNodes);

    std::vector<int> createEulerianCircuit(Graph& graph);

    void dfsEulerianCircuit(std::vector<std::vector<double>>& adjacencyMatrix, int node, std::vector<int>& eulerianCircuit);

    void createMinimumSpanningTree(Graph& graph, Graph& mst);

    void combineEdges(Graph& mst, const std::vector<int>& matching);

    int getMinimumKeyIndex(const Graph& graph, const std::vector<bool>& visited, const std::vector<double>& key);

    std::vector<int> tspHeuristic(Graph& graph);
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