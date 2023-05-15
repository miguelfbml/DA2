//
// Created by migue on 13/03/2023.
//

#ifndef DA1_MENU_H
#define DA1_MENU_H



class Menu {
public:
    /*!
     * constructor creates a new Menu object
     */
    Menu();

    /*!
     * prints a organized menu to interact with the user
     * @param network the network with the information and functions to be executed after selection
     */
    void runMenu();

    /*!
     * prints a separator on the menu to make the menu more organized
     */
    void menuSeparator();


};



#endif //DA1_MENU_H
