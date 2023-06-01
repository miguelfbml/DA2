//
// Created by migue on 13/03/2023.
//

#include <iostream>
#include "Menu.h"
#include "Graf.h"


using namespace std;

Menu::Menu(){}

void Menu::menuSeparator() {
    cout << endl << endl;
    for (int i = 0; i <= 69; i++) { cout << "="; }
    cout << endl;
}


void Menu::runMenu() {
    Graph *graph = new Graph();
    int option;


    while (true) {
        menuSeparator();

        cout << "" << endl <<
             endl;
        cout << "" << endl <<
             endl;

        cout << "exit                                                             [0]" <<
             endl;
        cout << "TSP for toygraph                                                 [1]" <<
             endl;
        cout << "TSP for Extra Fully Connected Graphs                             [2]" <<
             endl;
        cout << "        [3]" <<
             endl;
        cout << "   [4]" <<
             endl;
        cout << "   [5]" <<
             endl;
        cout << "        [6]" <<
             endl;
        cout << "    [7]" <<
             endl;
        cout << "            [8]" <<
             endl;

        cout << "Insert the number correspondent to your option: ";
        cin >> option;


        menuSeparator();

        if (option == 0) {
            exit(0);

        } else if (option == 1) {
            int option2;
            while (true) {
                menuSeparator();
                cout << "shipping           [1]" <<
                     endl;
                cout << "stadiums           [2]" <<
                     endl;
                cout << "tourism            [3]" <<
                    endl;

                cout << "Choose a file" << endl;
                cin >> option2;

                if (option2 == 1){

                    string file = "../Data/Toy-Graphs/shipping.csv";
                    int n = Graph::countNodes(file);

                    Graph::populateToyNodes(file, n);

                }

                if (option2 == 2){

                    string file = "../Data/Toy-Graphs/stadiums.csv";
                    int n = Graph::countNodes(file);

                    Graph::populateToyNodes(file, n);

                }

                if (option2 == 3){

                    string file = "../Data/Toy-Graphs/tourism.csv";
                    int n = Graph::countNodes(file);

                    Graph::populateToyNodes(file, n);

                }


            }



        } else if (option == 2) {

            int option3;
            while (true) {
                menuSeparator();
                cout << "shipping           [1]" <<
                     endl;
                cout << "stadiums           [2]" <<
                     endl;
                cout << "tourism            [3]" <<
                     endl;

                cout << "Choose a file" << endl;
                cin >> option3;

                if (option3 == 1){

                    string file = "../Data/Extra_Fully_Connected_Graphs/edges_25.csv";
                    int n = Graph::countNodes(file);

                    Graph::populateToyNodes(file, n);

                }

                if (option3 == 2){

                    string file = "../Data/Toy-Graphs/stadiums.csv";
                    int n = Graph::countNodes(file);

                    Graph::populateToyNodes(file, n);

                }

                if (option3 == 3){

                    string file = "../Data/Toy-Graphs/tourism.csv";
                    int n = Graph::countNodes(file);

                    Graph::populateToyNodes(file, n);

                }


            }








        } else if (option == 3) {
            int option2;
            while (true) {
                menuSeparator();
                cout << "shipping           [1]" <<
                     endl;
                cout << "stadiums           [2]" <<
                     endl;
                cout << "tourism            [3]" <<
                     endl;

                cout << "Choose a file" << endl;
                cin >> option2;

                if (option2 == 1){

                    string file_e = "../Data/Real-world Graphs/graph1/edges.csv";
                    string file_n = "../Data/Real-world Graphs/graph1/nodes.csv";

                    graph->populateGraph_nodes(*graph, file_n);
                    graph->populateGraph_edges(*graph, file_e);

                    vector<vector<double>> aux = graph->createAdjacencyMatrix(true);

                    double distfinal = 0;

                    vector<int> final = graph->tspTriangularApproximation(aux, distfinal);

                    for(int a : final){
                        cout << a << " - ";
                    }
                    cout << endl;

                    cout << "Dist: " << distfinal << "m" << endl;
                }

                if (option2 == 2){

                    string file_e = "../Data/Real-world Graphs/graph2/edges.csv";
                    string file_n = "../Data/Real-world Graphs/graph2/nodes.csv";

                    graph->populateGraph_nodes(*graph, file_n);
                    graph->populateGraph_edges(*graph, file_e);


                    vector<vector<double>> aux = graph->createAdjacencyMatrix();

                    /*
                    vector<vector<double>> aux;
                    vector<double> a1;
                    a1.push_back(0);
                    a1.push_back(10);
                    a1.push_back(15);
                    a1.push_back(20);
                    vector<double> a2;
                    a2.push_back(10);
                    a2.push_back(0);
                    a2.push_back(35);
                    a2.push_back(25);
                    vector<double> a3;
                    a3.push_back(15);
                    a3.push_back(35);
                    a3.push_back(0);
                    a3.push_back(30);
                    vector<double> a4;
                    a4.push_back(20);
                    a4.push_back(25);
                    a4.push_back(30);
                    a4.push_back(0);
                    aux.push_back(a1);
                    aux.push_back(a2);
                    aux.push_back(a3);
                    aux.push_back(a4);
                    */



                    double distfinal = 0;

                    vector<int> final = graph->tspTriangularApproximation(aux , distfinal);

                    for(auto a : final){
                        cout << a << " - ";
                    }
                    cout << endl;

                    cout << "Dist: " << distfinal << "km" << endl;
                }

                if (option2 == 3){

                    string file_e = "../Data/Real-world Graphs/graph3/edges.csv";
                    string file_n = "../Data/Real-world Graphs/graph3/nodes.csv";

                    graph->populateGraph_nodes(*graph, file_n);
                    graph->populateGraph_edges(*graph, file_e);



                    vector<vector<double>> aux = graph->createAdjacencyMatrix();

                    double distfinal = 0;

                    vector<int> final = graph->tspTriangularApproximation(aux , distfinal);

                    for(auto a : final){
                        cout << a << " - ";
                    }
                    cout << endl;

                    cout << "Dist: " << distfinal << "km" << endl;

                }

            }
        } else if (option == 4) {
            string file = "../Data/Toy-Graphs/tourism.csv";
            graph->populateGraph_edges(*graph, file);
            int n = Graph::countNodes(file);
            double distfinal = 0;
            vector<vector<double>> aux = graph->createAdjacencyMatrix(true, n);

            vector<int> final = graph->tspTriangularApproximation(aux , distfinal, n);

            for(auto a : final){
                cout << a << " - ";
            }
            cout << endl;

            cout << "Dist: " << distfinal << "m" << endl;


        } else if (option == 5) {

            
            

        } else if (option == 6) {
            
            


        } else if (option == 7) {
            
            


        } else if (option == 8) {
            
            

        }
    }

}
