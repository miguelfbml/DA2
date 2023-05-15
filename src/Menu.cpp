//
// Created by migue on 13/03/2023.
//

#include <iostream>
#include "Menu.h"


using namespace std;

Menu::Menu(){}

void Menu::menuSeparator() {
    cout << endl << endl;
    for (int i = 0; i <= 69; i++) { cout << "="; }
    cout << endl;
}


void Menu::runMenu() {
    int option;


    while (true) {
        menuSeparator();

        cout << "" << endl <<
             endl;
        cout << "" << endl <<
             endl;

        cout << "                                                               [0]" <<
             endl;
        cout << "                                [1]" <<
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
