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

        cout << endl << endl;

        cout << "Exit                                                             [0]" <<
             endl;
        cout << "Backtracking Algorithm                                           [1]" <<
             endl;
        cout << "Triangular Approximation Heuristic                               [2]" <<
             endl;
        cout << "Other Heuristics                                                 [3]" <<
             endl;

        cout << endl << "Insert the number correspondent to your option: ";
        cin >> option;


        menuSeparator();

        if (option == 0) {
            exit(0);

        }
        else if (option == 1) {
            int option2;
            while (true) {
                menuSeparator();
                cout << endl;
                cout << "Toy Graphs                                                       [1]" <<
                     endl;
                cout << "Real World Graphs                                                [2]" <<
                     endl;
                cout << "Extra Fully Connected Graphs                                     [3]" <<
                     endl;

                cout << endl << "Insert the number correspondent to your option: ";
                cin >> option2;
                cout << endl;

                if (option2 == 1){

                    int option3;
                    while (true){
                        menuSeparator();
                        cout << endl;
                        cout << "Shipping                                                         [1]" <<
                             endl;
                        cout << "Stadiums                                                         [2]" <<
                             endl;
                        cout << "Tourism                                                          [3]" <<
                             endl;

                        cout << endl << "Choose a file: ";
                        cin >> option3;
                        cout << endl;


                        if (option3 == 1){
                            string file = "../Data/Toy-Graphs/shipping.csv";
                            int n = Graph::countNodes(file);

                            Graph::populateToyNodes(file, n);
                            break;
                        }

                        else if (option3 == 2){
                            string file = "../Data/Toy-Graphs/stadiums.csv";
                            int n = Graph::countNodes(file);

                            Graph::populateToyNodes(file, n);
                            break;
                        }
                        else if (option3 == 3){
                            string file = "../Data/Toy-Graphs/tourism.csv";
                            int n = Graph::countNodes(file);

                            Graph::populateToyNodes(file, n);
                            break;
                        }
                    }

                }
                else if (option2 == 2){
                    int option3;
                    while (true){
                        menuSeparator();
                        cout << endl;
                        cout << "Graph 1                                                          [1]" <<
                             endl;
                        cout << "Graph 2                                                          [2]" <<
                             endl;
                        cout << "Graph 3                                                          [3]" <<
                             endl;

                        cout << endl << "Choose a file: ";
                        cin >> option3;
                        cout << endl;


                        if (option3 == 1){
                            //chamar ficheiro e funcoes

                            break;
                        }

                        else if (option3 == 2){
                            //chamar ficheiro e funcoes

                            break;
                        }
                        else if (option3 == 3){
                            //chamar ficheiro e funcoes

                            break;
                        }
                    }
                }

                else if (option2 == 3){
                    int option3;
                    while (true){
                        menuSeparator();
                        cout << endl;
                        cout << "Edges 25                                                         [1]" <<
                             endl;
                        cout << "Edges 50                                                         [2]" <<
                             endl;
                        cout << "Edges 75                                                         [3]" <<
                             endl;

                        cout << endl << "Choose a file: ";
                        cin >> option3;
                        cout << endl;


                        if (option3 == 1){
                            string file = "../Data/Extra_Fully_Connected_Graphs/edges_25.csv";
                            int n = Graph::countNodes(file);

                            Graph::populateToyNodes(file, n);
                            break;
                        }

                        else if (option3 == 2){
                            string file = "../Data/Extra_Fully_Connected_Graphs/edges_50.csv";
                            int n = Graph::countNodes(file);

                            Graph::populateToyNodes(file, n);
                            break;
                        }
                        else if (option3 == 3){
                            string file = "../Data/Extra_Fully_Connected_Graphs/edges_100.csv";
                            int n = Graph::countNodes(file);

                            Graph::populateToyNodes(file, n);
                            break;
                        }
                    }
                }
                break;


            }

        }
        else if (option == 2) {

            int option2;
            while (true) {
                menuSeparator();
                cout << endl;
                cout << "Toy Graphs                                                       [1]" <<
                     endl;
                cout << "Real World Graphs                                                [2]" <<
                     endl;
                cout << "Extra Fully Connected Graphs                                     [3]" <<
                     endl;

                cout << endl << "Insert the number correspondent to your option: ";
                cin >> option2;
                cout << endl;

                if (option2 == 1){

                    int option3;
                    while (true){
                        menuSeparator();
                        cout << endl;
                        cout << "Shipping                                                         [1]" <<
                             endl;
                        cout << "Stadiums                                                         [2]" <<
                             endl;
                        cout << "Tourism                                                          [3]" <<
                             endl;

                        cout << endl << "Choose a file: ";
                        cin >> option3;
                        cout << endl;


                        if (option3 == 1){
                            string file = "../Data/Toy-Graphs/shipping.csv";

                            //chamar funcao

                            break;
                        }

                        else if (option3 == 2){
                            string file = "../Data/Toy-Graphs/stadiums.csv";

                            //chamar funcao

                            break;
                        }
                        else if (option3 == 3){
                            string file = "../Data/Toy-Graphs/tourism.csv";

                            //chamar funcao

                            break;
                        }
                    }

                }
                else if (option2 == 2){
                    int option3;
                    while (true){
                        menuSeparator();
                        cout << endl;
                        cout << "Graph 1                                                          [1]" <<
                             endl;
                        cout << "Graph 2                                                          [2]" <<
                             endl;
                        cout << "Graph 3                                                          [3]" <<
                             endl;

                        cout << endl << "Choose a file: ";
                        cin >> option3;
                        cout << endl;


                        if (option3 == 1){
                            string file_e = "../Data/Real-world Graphs/graph1/edges.csv";
                            string file_n = "../Data/Real-world Graphs/graph1/nodes.csv";

                            graph->populateGraph_nodes(*graph, file_n);
                            graph->populateGraph_edges(*graph, file_e);

                            vector<int> caminho = graph->tspHeuristic(*graph);

                            for(auto a : caminho){
                                cout << a << " - ";
                            }
                            cout << endl;

                            break;
                        }

                        else if (option3 == 2){
                            //chamar ficheiro e funcoes

                            break;
                        }
                        else if (option3 == 3){
                            //chamar ficheiro e funcoes

                            break;
                        }
                    }

                }

                else if (option2 == 3){
                    int option3;
                    while (true){
                        menuSeparator();
                        cout << endl;
                        cout << "Edges 25                                                         [1]" <<
                             endl;
                        cout << "Edges 50                                                         [2]" <<
                             endl;
                        cout << "Edges 75                                                         [3]" <<
                             endl;

                        cout << endl << "Choose a file: ";
                        cin >> option3;
                        cout << endl;


                        if (option3 == 1){
                            string file = "../Data/Extra_Fully_Connected_Graphs/edges_25.csv";

                            //chamar funcao

                            break;
                        }

                        else if (option3 == 2){
                            string file = "../Data/Extra_Fully_Connected_Graphs/edges_50.csv";

                            //chamar funcao

                            break;
                        }
                        else if (option3 == 3){
                            string file = "../Data/Extra_Fully_Connected_Graphs/edges_100.csv";

                            //chamar funcao

                            break;
                        }

                    }

                }

                break;


            }

        }
        else if (option == 3) {
            int option2;
            while (true) {
                menuSeparator();
                cout << endl;
                cout << "Toy Graphs                                                       [1]" <<
                     endl;
                cout << "Real World Graphs                                                [2]" <<
                     endl;
                cout << "Extra Fully Connected Graphs                                     [3]" <<
                     endl;

                cout << endl << "Insert the number correspondent to your option: ";
                cin >> option2;
                cout << endl;

                if (option2 == 1){

                    int option3;
                    while (true){
                        menuSeparator();
                        cout << endl;
                        cout << "Shipping                                                         [1]" <<
                             endl;
                        cout << "Stadiums                                                         [2]" <<
                             endl;
                        cout << "Tourism                                                          [3]" <<
                             endl;

                        cout << endl << "Choose a file: ";
                        cin >> option3;
                        cout << endl;


                        if (option3 == 1){
                            string file = "../Data/Toy-Graphs/shipping.csv";

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

                            break;
                        }

                        else if (option3 == 2){
                            string file = "../Data/Toy-Graphs/stadiums.csv";

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

                            break;
                        }
                        else if (option3 == 3){
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

                            break;
                        }
                    }

                }
                else if (option2 == 2){
                    int option3;
                    while (true){
                        menuSeparator();
                        cout << endl;
                        cout << "Graph 1                                                          [1]" <<
                             endl;
                        cout << "Graph 2                                                          [2]" <<
                             endl;
                        cout << "Graph 3                                                          [3]" <<
                             endl;

                        cout << endl << "Choose a file: ";
                        cin >> option3;
                        cout << endl;


                        if (option3 == 1){
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

                            break;
                        }

                        else if (option3 == 2){
                            string file_e = "../Data/Real-world Graphs/graph2/edges.csv";
                            string file_n = "../Data/Real-world Graphs/graph2/nodes.csv";

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

                            break;
                        }
                        else if (option3 == 3){
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

                            break;
                        }
                    }

                }

                else if (option2 == 3){
                    int option3;
                    while (true){
                        menuSeparator();
                        cout << endl;
                        cout << "Edges 25                                                         [1]" <<
                             endl;
                        cout << "Edges 50                                                         [2]" <<
                             endl;
                        cout << "Edges 75                                                         [3]" <<
                             endl;

                        cout << endl << "Choose a file: ";
                        cin >> option3;
                        cout << endl;


                        if (option3 == 1){
                            string file = "../Data/Extra_Fully_Connected_Graphs/edges_25.csv";

                            //chamar funcao

                            break;
                        }

                        else if (option3 == 2){
                            string file = "../Data/Extra_Fully_Connected_Graphs/edges_50.csv";

                            //chamar funcao

                            break;
                        }
                        else if (option3 == 3){
                            string file = "../Data/Extra_Fully_Connected_Graphs/edges_100.csv";

                            //chamar funcao

                            break;
                        }

                    }

                }

                break;


            }
        }
    }

}
