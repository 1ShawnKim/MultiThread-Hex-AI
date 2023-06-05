//
//  graph.hpp
//   including a basic graph class and its derivative for Hex Game
//             a shortest path algorithm and a MST algorithm using the class.
//  Created by Shawn Kim on 6/5/23.
//  reference 1: https://en.wikipedia.org/wiki/Dijkstra's_algorithm
//  reference 2: https://en.wikipedia.org/wiki/Minimum_spanning_tree
#ifndef graph_hpp
#define graph_hpp

#include <stdio.h>
#include <vector>
#include <queue>
#include <ctime>
#include <iostream>
#include <iomanip>
#include <fstream>
#include <random>
#include <thread>
#include <future>

using namespace std;

typedef enum algoID{Dijkstra, Kruskal, Prim} algoID;
enum class Hex_Color:char {EMPTY=0, RED, BLUE};

class Graph {
protected:
    vector<Hex_Color> gnode;
    vector<vector<pair<long,int>>> edge; // using matrix representation for graph (2D vector)
    bool undirected; // Undrected graph is default;
    bool noloop;
    int num_color;
    
public:
    Graph(int num_V = 50, bool ud = true, bool nl = true, int num = 3):gnode(num_V, Hex_Color::EMPTY),edge(vector<vector<pair<long,int>>>(num_V, vector<pair<long,int>>(num_V,make_pair(numeric_limits<long>::max(),0)))),undirected(ud),noloop(nl), num_color(num) {};// constructor
    Graph(string fname); // constructor for reading from a external file
    ~Graph(){};//distructor
    // member getter & settor?
    
    int get_num_node(Hex_Color c = Hex_Color::EMPTY) {
        int num = 0;
        for (auto node:gnode) if(node == c) num++;
        return num;
    };
    
    bool adjacent(int x, int y){return (edge[x][y].first != 0);};//: tests whether there is an edge from node x to node y.
    auto neighbors(int x);//: lists all nodes y such that there is an edge from x to y.
    auto neighbors(int x, vector<Hex_Color> & grnode);
    
    void add(int x, int y, long value){edge[x][y].first = value;};//: adds to G the edge from x to y, if it is not there.
    void remove(int x, int y){edge[x][y].first = numeric_limits<long>::max();};//: removes the edge from x to y, if it is there.
    
 //   long get_node_value(long x){return node[x];};//: returns the value associated with the node x.
 //   void set_node_value(int x, long value){node[x] = value;};//: sets the value associated with the node x to a.
    auto get_edge_value(int x, int y){return edge[x][y];};//: returns the value associated to the edge (x,y).
    void set_edge_value(int x, int y, long first, int second = 0){edge[x][y].first = first;edge[x][y].second = second;};//: sets the value associated to the edge (x,y) to v.
    
    long randomize(double density, pair<long,long> distance_range); // make itself random with 'density' and 'distance_range'
    
    void display_edges();
    
    bool get_shortest_path_tree(int source, vector<pair<int,long>> &shortest_path_tree, algoID id = Dijkstra); // result : true = having no isolated node. false = having any isolated nodes
    bool get_shortest_path_tree(int source,vector<Hex_Color> & grnode,vector<pair<int,long>> &shortest_path_tree, algoID id = Dijkstra);
    long long get_MST(int seed, vector<pair<int,long>> &MST, algoID id = Prim); // the return value is the cost.
//    long get_MST(int seed, long density, vector<pair<int,long>> &MST, algoID id = Prim); // for debug
 };

class HEX_Graph : Graph {
private:
    const int bsize; //board size, number of nodes in a row
    friend int montecarlo(HEX_Graph *hex, vector<int> random_sequence, int cursor, int num_iter);
public:
    HEX_Graph(int size = 7);
    ~HEX_Graph(){};
    void display_HEX_board();
    Hex_Color get_node_value(int i, int j) {return gnode[i*bsize+j];};
    void set_node_value(int i, int j, Hex_Color c) { gnode[i*bsize+j] = c;};
    Hex_Color determine_who_won();
    Hex_Color determine_who_won(vector<Hex_Color> &grnode);
    void autoplay(const Hex_Color &hc, const int NUM_THREADS, const int MC_SIZE);
    bool get_user_input(const Hex_Color &hc);
};

class PriorityQueue {
private:
    vector<pair<int,long>> c;
public:
    PriorityQueue(int num=0) { for(int i = 0 ; i < num; i++) c.push_back(make_pair(i,numeric_limits<long>::max()));};
    PriorityQueue(vector<pair<int,long>> init) { for(auto comp: init) c.push_back(make_pair(comp.first ,comp.second));};
    
    ~PriorityQueue(){};// Distructor
    bool chgPriority(int node, long priority);//: changes the priority (node value) of queue element.
    bool contains(int node);//: does the queue contain queue_element.
    long getPriority(int node);
    void pop() {c.pop_back();};
    bool empty() {return c.empty();};
    pair<int,long> bottom() {return c.back();};
    void push_back(pair<int,long> elm) {c.push_back(elm);};
};

#endif /* graph_hpp */
