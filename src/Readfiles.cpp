//
// Created by migue on 13/03/2023.
//

#include "Readfiles.h"


Readfiles::Readfiles() {

}


void Readfiles::readStations() {
    ifstream infile("../Data/stations.csv");
    string curLine;
    unordered_map<string, int> table;
    unordered_map<string, int> municipalities;
    unordered_map<string, int> districts;

    
    int i = 0;
    string source;
    string destiny;
    string dist;
 
    getline(infile, curLine); //livrar a primeira linha que é só as designaçoes


    while(getline(infile, curLine)){


        stringstream inputString(curLine);

        getline(inputString, source, ',');
        getline(inputString, destiny, ',');
        getline(inputString, dist, ',');
    



        auto it = table.find(source);
        if (it != table.end()) {
            continue;
        } else {
            //nao existe elemento
        }


        districts[destiny] = 0;
        municipalities[dist] = 0;


        table[source] = i;
        i++;
    }

/*
    network->setGraph(g);
    network->setHash(table);
    network->setMunicipalities(municipalities);
    network->setDistricts(districts);
*/

    infile.close();
}

