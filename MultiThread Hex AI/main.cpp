//  main.cpp
//  MultiThread Hex AI
//  A simple HEX game using multi-thread Monte Carlo AI.
//
//  Created by 1ShawnKim@Github on 6/5/23.
//
#include "graph.hpp"

const int BD_SIZE = 7;
const int MC_SIZE = 20000;
const int NUM_THREADS = 8;

int main(int argc, const char * argv[]) {
    
    Hex_Color winner, computer, human, next_turn;
    string answer;
    
    while(1) {
        HEX_Graph hex(BD_SIZE);
        hex.display_HEX_board();
    
        cout << "Do you want to play first?(Y/N) or put 'x' to end the game: ";
        cin >> answer;
        if(answer == "x") return 0;
        next_turn = Hex_Color::BLUE;
        human = ((answer == "Y" || answer == "y")?Hex_Color::BLUE:Hex_Color::RED);
        computer = ((answer == "Y" || answer == "y")?Hex_Color::RED:Hex_Color::BLUE);
    
        do {
            if(next_turn == human) hex.get_user_input(human);
            else hex.autoplay(computer, NUM_THREADS, MC_SIZE);
            next_turn = (next_turn == human?computer:human);
            winner = hex.determine_who_won();
        } while (winner == Hex_Color::EMPTY);
    
        if( winner == human) cout << "Congrats! You win!\n";
        else cout << "Sorry, You loose...\n";
    }

    return 0;
}

