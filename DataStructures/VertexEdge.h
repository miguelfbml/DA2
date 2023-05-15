

#ifndef DA_TP_CLASSES_VERTEX_EDGE
#define DA_TP_CLASSES_VERTEX_EDGE

#include <iostream>
#include <vector>
#include <queue>
#include <limits>
#include <algorithm>
#include <string>
#include "../DataStructures/MutablePriorityQueue.h"

using namespace std;

class Edge;

#define INF std::numeric_limits<double>::max()

/************************* Vertex  **************************/

class Vertex {
public:
    Vertex(int id);
    Vertex();
    Vertex(int id, string name, string district, string municipality, string township, string line);
    bool operator<(Vertex & vertex) const; // // required by MutablePriorityQueue

    int getId() const;
    std::vector<Edge *> getAdj() const;
    bool isVisited() const;
    bool isProcessing() const;
    unsigned int getIndegree() const;
    double getDist() const;
    Edge *getPath() const;
    std::vector<Edge *> getIncoming() const;

    void setId(int info);
    void setVisited(bool visited);
    void setProcesssing(bool processing);
    void setIndegree(unsigned int indegree);
    void setDist(double dist);
    void setPath(Edge *path);
    Edge * addEdge(Vertex *dest, double w);
    Edge * addEdgeWithParameters(Vertex *dest, int cap, string serv);
    bool removeEdge(int destID);

    void setName(string name);
    void setDistrict(string dist);
    void setMuni(string muni);
    void setTown(string town);
    void setLine(string line);

    string getName() const;
    string getDistr() const;
    string getMuni() const;

    friend class MutablePriorityQueue<Vertex>;
protected:
    int id;                // identifier
    std::vector<Edge *> adj;  // outgoing edges

    // auxiliary fields
    bool visited = false; // used by DFS, BFS, Prim ...
    bool processing = false; // used by isDAG (in addition to the visited attribute)
    unsigned int indegree; // used by topsort
    double dist = 0;
    string name;
    string district;
    string municipality;
    string township;
    string line;
    Edge *path = nullptr;


    std::vector<Edge *> incoming; // incoming edges

    int queueIndex = 0; 		// required by MutablePriorityQueue and UFDS
    Edge *addEdgeWithParametersNoIncoming(Vertex *d, int c, string serv);
};

/********************** Edge  ****************************/

class Edge {
public:
    Edge(Vertex *orig, Vertex *dest, double w);
    Edge();
    Edge(Vertex* stationA, Vertex* stationB, int capacity, string service);

    Vertex * getDest() const;
    double getWeight() const;
    bool isSelected() const;
    Vertex * getOrig() const;
    Edge *getReverse() const;
    double getFlow() const;
    Vertex* getStationA() const;
    Vertex* getStationB() const;
    int getCapacity() const;
    string getService() const;

    void setSelected(bool selected);
    void setReverse(Edge *reverse);
    void setFlow(double flow);
    void setStationA(Vertex* stationA);
    void setStationB(Vertex* stationB);
    void setCapacity(int capacity);
    void setService(string service);
    void setWeight(double weight);
protected:
    Vertex * dest; // destination vertex
    double weight; // edge weight, can also be used for capacity

    // auxiliary fields
    bool selected = false;

    // used for bidirectional edges
    Vertex *orig;
    Edge *reverse = nullptr;

    double flow; // for flow-related problems

    Vertex* stationA;
    Vertex* stationB;
    int capacity;
    string service;
};

#endif /* DA_TP_CLASSES_VERTEX_EDGE */