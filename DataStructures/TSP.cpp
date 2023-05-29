#include "TSP.h"

vector <int> TSPSolver::solveTSP() {
    int numNodes = graph.getNumNodes();
    vector<int> path;
    vector<int> nodes = graph.getNodes();
    unordered_set<int> visited;
    double minDistance = numeric_limits<double>::max();

    backtrack(0, path, visited, minDistance, nodes);

    return bestPath;
}

void TSPSolver::backtrack(int current, vector<int>& path, unordered_set<int>& visited,
                double& minDistance, const vector<int>& nodes) {
    if (visited.size() == nodes.size() && graph.calculateDistance(current, 0) < minDistance) {
        bestPath = path;
        minDistance = graph.calculateDistance(current, 0);
        return;
    }

    for (int neighbor : graph.getNeighbors(current)) {
        if (visited.find(neighbor) == visited.end()) {
            double distance = graph.calculateDistance(current, neighbor);

            if (path.size() + 1 == nodes.size() && graph.calculateDistance(neighbor, 0) < minDistance) {
                path.push_back(neighbor);
                path.push_back(0);
                bestPath = path;
                minDistance = graph.calculateDistance(neighbor, 0);
                path.pop_back();
                path.pop_back();
                continue;
            }

            visited.insert(neighbor);
            path.push_back(neighbor);
            backtrack(neighbor, path, visited, minDistance, nodes);
            visited.erase(neighbor);
            path.pop_back();
        }
    }
}