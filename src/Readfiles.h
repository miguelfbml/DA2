//
// Created by migue on 13/03/2023.
//

#ifndef DA1_READFILES_H
#define DA1_READFILES_H

#include <string>

#include <sstream>
#include <fstream>
#include <unordered_map>
using namespace std;


class Readfiles {

public:
    /**
     * Default constructor
     * @brief Complexidade O(1)
     */
    Readfiles();

    /**
     * Reads data from file stations.csv
     * @param network reads data into network object
     */
    static void readStations();

    /**
     * reads data from file network.csv
     * @param network reads data into network object
     */
    static void readNetwork();
};



#endif //DA1_READFILES_H
