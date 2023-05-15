

#include "VertexEdge.h"

/************************* Vertex  **************************/

Vertex::Vertex(int id): id(id) {}

/*
 * Auxiliary function to add an outgoing edge to a vertex (this),
 * with a given destination vertex (d) and edge weight (w).
 */
Edge * Vertex::addEdge(Vertex *d, double w) {
    auto newEdge = new Edge(this, d, w);
    adj.push_back(newEdge);
    d->incoming.push_back(newEdge);
    return newEdge;
}


Edge * Vertex::addEdgeWithParameters(Vertex *d, int c, string serv) {
    auto newEdge = new Edge(this, d, c, serv);
    adj.push_back(newEdge);
    d->incoming.push_back(newEdge);
    return newEdge;
}

Edge * Vertex::addEdgeWithParametersNoIncoming(Vertex *d, int c, string serv) {
    auto newEdge = new Edge(this, d, c, serv);
    adj.push_back(newEdge);
    return newEdge;
}

/*
 * Auxiliary function to remove an outgoing edge (with a given destination (d))
 * from a vertex (this).
 * Returns true if successful, and false if such edge does not exist.
 */
bool Vertex::removeEdge(int destID) {
    bool removedEdge = false;
    auto it = adj.begin();
    while (it != adj.end()) {
        Edge *edge = *it;
        Vertex *dest = edge->getDest();
        if (dest->getId() == destID) {
            it = adj.erase(it);
            // Also remove the corresponding edge from the incoming list
            auto it2 = dest->incoming.begin();
            while (it2 != dest->incoming.end()) {
                if ((*it2)->getOrig()->getId() == id) {
                    it2 = dest->incoming.erase(it2);
                }
                else {
                    it2++;
                }
            }
            delete edge;
            removedEdge = true; // allows for multiple edges to connect the same pair of vertices (multigraph)
        }
        else {
            it++;
        }
    }
    return removedEdge;
}

bool Vertex::operator<(Vertex & vertex) const {
    return this->dist < vertex.dist;
}

/*
void Vertex::delVertex(int id){

}
*/

int Vertex::getId() const {
    return this->id;
}

std::vector<Edge*> Vertex::getAdj() const {
    return this->adj;
}

bool Vertex::isVisited() const {
    return this->visited;
}

bool Vertex::isProcessing() const {
    return this->processing;
}

unsigned int Vertex::getIndegree() const {
    return this->indegree;
}

double Vertex::getDist() const {
    return this->dist;
}

Edge *Vertex::getPath() const {
    return this->path;
}

std::vector<Edge *> Vertex::getIncoming() const {
    return this->incoming;
}

void Vertex::setId(int id) {
    this->id = id;
}

void Vertex::setVisited(bool visited) {
    this->visited = visited;
}

void Vertex::setProcesssing(bool processing) {
    this->processing = processing;
}

void Vertex::setIndegree(unsigned int indegree) {
    this->indegree = indegree;
}

void Vertex::setDist(double dist) {
    this->dist = dist;
}

void Vertex::setPath(Edge *path) {
    this->path = path;
}

/********************** Edge  ****************************/

Edge::Edge(Vertex *orig, Vertex *dest, double w): orig(orig), dest(dest), weight(w) {}

Edge::Edge(Vertex *stationA, Vertex *stationB, int capacity, string service) {
    this->orig = stationA;
    this->dest = stationB;
this->stationA = stationA;
this->stationB = stationB;
this->capacity = capacity;
this->service = service;
this->weight = capacity;
}


Vertex * Edge::getDest() const {
    return this->dest;
}

double Edge::getWeight() const {
    return this->weight;
}

Vertex * Edge::getOrig() const {
    return this->orig;
}

Edge *Edge::getReverse() const {
    return this->reverse;
}

bool Edge::isSelected() const {
    return this->selected;
}

double Edge::getFlow() const {
    return flow;
}

Vertex* Edge::getStationA() const {
    return this->stationA;
}

Vertex* Edge::getStationB() const {
    return this->stationB;
}

int Edge::getCapacity() const {
    return this->capacity;
}

string Edge::getService() const {
    return this->service;
}

void Edge::setSelected(bool selected) {
    this->selected = selected;
}

void Edge::setReverse(Edge *reverse) {
    this->reverse = reverse;
}

void Edge::setFlow(double flow) {
    this->flow = flow;
}

void Vertex::setName(std::string name) {
    this->name = name;
}

void Vertex::setDistrict(std::string dist) {
    this->district = dist;
}

void Vertex::setMuni(std::string muni) {
    this -> municipality = muni;
}

void Vertex::setTown(std::string town) {
    this->township = town;
}

void Vertex::setLine(std::string line) {
    this->line = line;
}

void Edge::setStationA(Vertex* stationA){
    this->stationA = stationA;
}

void Edge::setStationB(Vertex* stationB) {
    this->stationB = stationB;
}

void Edge::setCapacity(int capacity) {
    this->capacity = capacity;
}

void Edge::setWeight(double weight){
    this->weight = weight;
}

void Edge::setService(string service) {
    this->service = service;
}

Vertex::Vertex(int id, std::string name, std::string district, std::string municipality, std::string township, std::string line) {
    this->id = id;
    this->name = name;
    this->district = district;
    this->municipality = municipality;
    this->township = township;
    this->line = line;
}



string Vertex::getName() const {
    return this->name;
}

string Vertex::getMuni() const {
    return this ->municipality;
}

string Vertex::getDistr() const {
    return this ->district;
}