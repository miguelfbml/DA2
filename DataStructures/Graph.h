

#ifndef DA_TP_CLASSES_GRAPH
#define DA_TP_CLASSES_GRAPH

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include "../DataStructures/MutablePriorityQueue.h"

#include "VertexEdge.h"

class Graph {
public:
    ~Graph();
    /*
    * Auxiliary function to find a vertex with a given ID.
    */
    Vertex *findVertex(const int &id) const;
    /*
     *  Adds a vertex with a given content or info (in) to a graph (this).
     *  Returns true if successful, and false if a vertex with that content already exists.
     */
    bool addVertex(const int &id);


    bool addVertexParameters(const int id, string name, string district, string municipality, string township, string line);

    /*
     * Adds an edge to a graph (this), given the contents of the source and
     * destination vertices and the edge weight (w).
     * Returns true if successful, and false if the source or destination vertex does not exist.
     */
    bool addEdge(const int &sourc, const int &dest, double w);
    bool addBidirectionalEdge(const int &sourc, const int &dest, double w);
    bool addEdgeWithParameters(const int &sourc, const int &dest, int cap, string serv);

    int getNumVertex() const;
    std::vector<Vertex *> getVertexSet() const;
    int findVertexId(string name);

//edmondsKarp
    int edmondsKarp(int source, int target);

    int edmondsKarp_opt(int source, int target);

    bool removeVertex(const int &id);


protected:
    std::vector<Vertex *> vertexSet;    // vertex set

    double ** distMatrix = nullptr;   // dist matrix for Floyd-Warshall
    int **pathMatrix = nullptr;   // path matrix for Floyd-Warshall

    /*
     * Finds the index of the vertex with a given content.
     */
    int findVertexIdx(const int &id) const;


    void testAndVisit(queue<Vertex *> &q, Edge *e, Vertex *w, double residual);

    bool findAugmentingPath(Vertex *s, Vertex *t);

    double findMinResidualAlongPath(Vertex *s, Vertex *t);

    void augmentFlowAlongPath(Vertex *s, Vertex *t, double f);

    bool findAugmentingPath_opt(Vertex *s, Vertex *t);

};

void deleteMatrix(int **m, int n);
void deleteMatrix(double **m, int n);

#endif /* DA_TP_CLASSES_GRAPH */