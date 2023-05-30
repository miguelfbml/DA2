#ifndef tsp_H
#define tsp_H

#include <vector>


float tspBT(const float **dists, unsigned int n, unsigned int path[]);
void tspBTRec(const float **dists, unsigned int n, unsigned int curIndex, float curDist, unsigned int curPath[], float &minDist, unsigned int path[]);

#endif 