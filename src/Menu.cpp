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
        cout << "          [2]" <<
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


                    string file = "../Data/Toy-Graphs/Toy-Graphs/shipping.csv";
                    int n = Graph::countNodes(file);


                    Graph::populateToyNodes(file, n);



                }





            }



        } else if (option == 2) {

            


        } else if (option == 3) {
            



        } else if (option == 4) {
            
            


        } else if (option == 5) {

            
            

        } else if (option == 6) {
            
            


        } else if (option == 7) {
            
            


        } else if (option == 8) {
            
            

        }
    }

}
