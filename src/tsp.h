#ifndef tsp_H
#define tsp_H

#include <vector>


unsigned int tspBT(const unsigned int **dists, unsigned int n, unsigned int path[]);
void tspBTRec(const unsigned int **dists, unsigned int n, unsigned int curIndex, unsigned int curDist, unsigned int curPath[], unsigned int &minDist, unsigned int path[]);

#endif 