#include <iostream>
#include <vector>
#include <unordered_set>
#include <limits>
#include "Graf.h"

using namespace std;

#ifndef DA_TP_CLASSES_TSP
#define DA_TP_CLASSES_TSP

class TSPSolver {
public:
    TSPSolver(Graph& graph) : graph(graph) {}

    vector<int> solveTSP();

private:
    Graph& graph;
    vector<int> bestPath;

    void backtrack(int current, vector<int>& path, unordered_set<int>& visited,
                   double& minDistance, const vector<int>& nodes);
};

#endif //DA_TP_CLASSES_TSP