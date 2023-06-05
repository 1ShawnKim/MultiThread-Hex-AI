//
//  graph.cpp
//   including a basic graph class and its derivative for Hex Game
//             a shortest path algorithm and a MST algorithm using the class.
//  Created by 1ShawnKim@Github on 6/5/23.
//  reference 1: https://en.wikipedia.org/wiki/Dijkstra's_algorithm
//  reference 2: https://en.wikipedia.org/wiki/Minimum_spanning_tree

#define NDEBUG_MAP
 
#include "graph.hpp"

double prob()
{
    return static_cast<double>(rand())/RAND_MAX;
}

Graph::Graph(string fname)
{
    
    ifstream inp(fname);
    
    if(inp.is_open()) {
        
        undirected = true; // Undrected graph is default;
        noloop = true;
        num_color = 0;
        
        int num_node;
        inp >> num_node;
        
        edge = vector<vector<pair<long,int>>>(num_node, vector<pair<long,int>>(num_node,make_pair(numeric_limits<long>::max(),0)));
        
        int x, y, cost;
        
        while (inp >> x >> y >> cost) {
            edge[x][y].first = static_cast<long>(cost);
        }
        
#ifndef NDEBUG_MAP
        display_edges();
#endif
    }
}

auto Graph::neighbors(int x)//: lists all nodes y such that there is an edge from x to y with the same color.
{
    vector<int> list;
    
    for(int i = 0; auto s_edge : edge[x]) {
        if(s_edge.first < numeric_limits<long>::max()&& (gnode[x] == gnode[i])) list.push_back(i);
        i++;
    }
    
    return list;
}

auto Graph::neighbors(int x, vector<Hex_Color> & grnode)//: lists all nodes y such that there is an edge from x to y with the same color.
{
    vector<int> list;
    
    for(int i = 0; auto s_edge : edge[x]) {
        if(s_edge.first < numeric_limits<long>::max()&& (grnode[x] == grnode[i])) list.push_back(i);
        i++;
    }
    
    return list;
}

long Graph::randomize(double density, pair<long,long> distance_range) // make itself random with 'density' and 'distance_range'
{
    srand(static_cast<unsigned int>(time(0)));
    double current_density = 0, inc = (undirected?2./static_cast<long>(edge.size()*edge.size()):1./static_cast<long>(edge.size()*edge.size()));
    
    //initialize edge vector with maxium cost

    for ( auto &edge_row : edge) {
        for ( auto &s_edge : edge_row) {
            s_edge.first = numeric_limits<long>::max();
            s_edge.second = 0;
        }
    }
    
    for(int i = 0; density > current_density ; i=++i%edge.size()){// i=i++%edge.size() is not correct and working.
        for(int j = (undirected?(noloop?i+1:i):0); j < edge.size() ; j++){
            if(prob() < density) {
                if(edge[i][j].first == numeric_limits<long>::max()) {
                    edge[i][j].first = 1.0;
                    if(density <= (current_density += inc))
                        break;
                }
            }
        }
    }

    for(int i = 0; i < edge.size() ; i++) {
        for(int j = (undirected?(noloop?i+1:i):0); j < edge.size() ; j++) {
            if(edge[i][j].first == 1.0)
            {
                edge[i][j] = make_pair(((distance_range.second - distance_range.first))*prob()+distance_range.first, rand()%num_color);
                if(undirected) edge[j][i] = edge[i][j]; // more efficient than exact range : if(undirected && i != j) edge[i][j] = edge[j][i];
            }
        }
    }
    
#ifndef NDEBUG_MAP
    // cout the edges
    display_edges();
#endif

    return current_density;
}

void Graph::display_edges()
{
    // cout the edges
    for(int i = 0; i < edge.size(); i++) {
        for(int j = 0; j < edge.size(); j++) {
            if(edge[i][j].first < numeric_limits<long>::max()) cout << setfill(' ') << setw(2) << fixed << static_cast<int>(edge[i][j].first) << "|";
            else cout << "  |";
        }
    cout << endl;
    }
}

bool Graph::get_shortest_path_tree(int source, vector<pair<int,long>> &shortest_path_tree, algoID id)
{
    int start = source;
    
    shortest_path_tree = vector<pair<int,long>>(gnode.size(), pair<int,long>(-1,numeric_limits<long>::max()));
    shortest_path_tree[source].first = source;
    shortest_path_tree[source].second = 0.0;

    PriorityQueue minPQ;
    for (int i = 0; auto node:gnode) {
        if(node == gnode[source]) minPQ.push_back(make_pair(i,numeric_limits<long>::max()));
        i++;
    }
    
    minPQ.chgPriority(source, 0);

    while (!minPQ.empty())
    {
        start = minPQ.bottom().first;
        long start_priority = minPQ.getPriority(start);
        
        if(start_priority == numeric_limits<long>::max()) { //Graph is broken.
            return false;
        }

        minPQ.pop();
        for(auto neighbor : neighbors(start))
        {
            if(minPQ.contains(neighbor))
            {
                long dist = start_priority + get_edge_value(start, neighbor).first;
            
                if(minPQ.chgPriority(neighbor, dist))
                {
                    shortest_path_tree[neighbor].first = start; // previous node
                    shortest_path_tree[neighbor].second = dist; // shortest path length from source.
                }
            }
        }
    }
    return true;
}

bool Graph::get_shortest_path_tree(int source,vector<Hex_Color> & grnode,vector<pair<int,long>> &shortest_path_tree, algoID id)
{
    int start = source;
    
    shortest_path_tree = vector<pair<int,long>>(grnode.size(), pair<int,long>(-1,numeric_limits<long>::max()));
    shortest_path_tree[source].first = source;
    shortest_path_tree[source].second = 0.0;
    
    PriorityQueue minPQ;
    for (int i = 0; auto node:grnode) {
        if(node == grnode[source]) minPQ.push_back(make_pair(i,numeric_limits<long>::max()));
        i++;
    }
    
    minPQ.chgPriority(source, 0);

    while (!minPQ.empty())
    {
        start = minPQ.bottom().first;
        long start_priority = minPQ.getPriority(start);
        
        if(start_priority == numeric_limits<long>::max())   return false; //Graph is broken.

        minPQ.pop();//?? need to eliminate bottom.
        
        for(auto neighbor : neighbors(start,grnode))
        {
            if(minPQ.contains(neighbor))
            {
                long dist = start_priority + get_edge_value(start, neighbor).first;
            
                if(minPQ.chgPriority(neighbor, dist))
                {
                    shortest_path_tree[neighbor].first = start; // previous node
                    shortest_path_tree[neighbor].second = dist; // shortest path length from source.
                }
            }
        }
    }
    return true;
}



long long Graph::get_MST(int seed, vector<pair<int,long>> &MST, algoID id)
{
    MST = vector<pair<int,long>>(get_num_node(), pair<int,long>(-1,numeric_limits<long>::max()));
    
    MST[seed].first = seed;
    MST[seed].second = 0;
        
    PriorityQueue minPQ = PriorityQueue(get_num_node());
    minPQ.chgPriority(seed, MST[seed].second);

    while (!minPQ.empty())
    {
        int start = minPQ.bottom().first;
        if(minPQ.getPriority(start) == numeric_limits<long>::max()) { //Graph is broken.
            return -1;
        }

        minPQ.pop();
        
        for(auto neighbor : neighbors(start))
        {
            if(minPQ.contains(neighbor))
            {
                long dist = get_edge_value(start, neighbor).first; // This is a main difference between Dijkstra and MST(Prim).
            
                if(minPQ.chgPriority(neighbor, dist))
                {
                    MST[neighbor].first = start; // previous node
                    MST[neighbor].second = dist; // shortest path length from source.
                }
            }
        }
    }
    
    long long cost = 0;
    for(auto node : MST) {
        
        cost += node.second;
    }
    
    return cost;
}

HEX_Graph::HEX_Graph(int size):bsize(size)
{
    undirected = false;
    noloop = true;
    num_color = 3;
    
    gnode = vector<Hex_Color>(bsize*bsize, Hex_Color::EMPTY);
    
    edge = vector<vector<pair<long,int>>>(bsize*bsize, vector<pair<long,int>>(bsize*bsize,make_pair(numeric_limits<long>::max(),0)));
    
    for (int i = 0; i < bsize; i++ ) {
        for (int j = 0; j < bsize; j++) {
            int pos = i*bsize+j;
            
            if(i >= 1) edge[pos][pos-bsize].first = 1;
            if(i >= 1 && j < bsize-1) edge[pos][pos-bsize+1].first = 1;
            if(j >= 1) edge[pos][pos-1].first = 1;
            if(j < bsize -1) edge[pos][pos+1].first = 1;
            if(i < bsize -1 && j >= 1) edge[pos][pos+bsize-1].first = 1;
            if(i < bsize -1) edge[pos][pos+bsize].first = 1;
        }
    }
#ifndef NDEBUG_MAP
    display_edges(); // cout the edges
#endif
    
}

void HEX_Graph::display_HEX_board()
{
    cout << "\n";
    for (int j = 0 ; j < bsize; j++) {
        cout << setw(4) << right <<j;
    }
    cout << "\n";
    
    for (int i = 0 ; i < bsize; i++) {
        cout << setw(3) << left << i;
        for (int j = 0; j < bsize; j++) {
            switch (gnode[bsize*i+j]) {
                case Hex_Color::EMPTY:
                    cout << ".";
                    break;
                case Hex_Color::BLUE:
                    cout << "X";
                    break;
                case Hex_Color::RED:
                    cout << "O";
                    break;
            }
            if (j != bsize -1) cout << " - ";
            else cout <<"\n";
        }
        
        if(i < bsize - 1) {
            cout << "    ";
            for (int j = 0; j < i ; j++) cout << "  ";
            for (int j = 1; j < bsize ; j++) cout << "\\ / ";
            cout << "\\\n  ";
            for (int j = 0; j < i ; j++) cout << "  ";
        }
    }
    cout << endl;
    
}

Hex_Color HEX_Graph::determine_who_won()
{
    Hex_Color winner = Hex_Color::EMPTY;
    
    //check for blue by using shortest path algorithm.
    for (int i = 0; i < bsize; i++)
    {
        if(gnode[i*bsize] == Hex_Color::BLUE) {
            vector<pair<int,long>> shortest_path_tree;
            get_shortest_path_tree(i*bsize, shortest_path_tree);
            for(int j = 0; j < bsize; j++)
            {
                if(shortest_path_tree[(j+1)*bsize-1].second < numeric_limits<long>::max()) {
                    winner = Hex_Color::BLUE;
                    break;
                }
            }
            
            if(winner == Hex_Color::BLUE) break;
        }
    }
    
    //check for red by using shortest path algorithm.
    if(winner == Hex_Color::EMPTY) {
        for (int i = 0; i < bsize; i++)
        {
            if(gnode[i] == Hex_Color::RED) {
                vector<pair<int,long>> shortest_path_tree;
                get_shortest_path_tree(i, shortest_path_tree);
                for(int j = 0; j < bsize; j++)
                {
                    if(shortest_path_tree[j+(bsize-1)*bsize].second < numeric_limits<long>::max()) {
                        winner = Hex_Color::RED;
                        break;
                    }
                }
                
                if(winner == Hex_Color::RED) break;
            }
        }
    }
    
    display_HEX_board();
    
    return winner;
}

Hex_Color HEX_Graph::determine_who_won(vector<Hex_Color> &grnode)
{
    vector<pair<int,long>> shortest_path_tree;
    Hex_Color winner = Hex_Color::BLUE;
    
    for (int i = 0; i < bsize; i++)
    {
        if(grnode[i] == Hex_Color::RED) {
            get_shortest_path_tree(i, grnode, shortest_path_tree);
            for(int j = 0; j < bsize; j++)
            {
                if(shortest_path_tree[j+(bsize-1)*bsize].second < numeric_limits<long>::max()) {
                    winner = Hex_Color::RED;
                    break;
                }
            }
                
            if(winner == Hex_Color::RED) break;
        }
    }
    
    return winner;
}

int montecarlo(HEX_Graph *hex, vector<int> random_sequence, int cursor, int num_iter) // Monte Carlo Simulation for multi parallel processings
{
    vector<Hex_Color> temp_gnode(hex->gnode);
    temp_gnode[cursor] = Hex_Color::RED;
    
    random_device rd;
    mt19937 g(rd());
    
    int count = 0;
    for(long k = 0; k < num_iter; k++) {
        shuffle(random_sequence.begin(),random_sequence.end(),g); //shuffle the sequence
        for(int j = 0; j < random_sequence.size(); j++) temp_gnode[random_sequence[j]] = (j%2==0?Hex_Color::BLUE:Hex_Color::RED);

        if( hex->determine_who_won(temp_gnode) == Hex_Color::RED) count++;
    }
    
    return count;
}

void HEX_Graph::autoplay(const Hex_Color &hc, const int NUM_THREADS, const int MC_SIZE) // by Monte Carlo Simulation
{
    cout << "It's computer's turn." <<endl;
    
    auto start_time = chrono::steady_clock::now();
    auto c_start = clock();
    
    vector<pair<int, int>> empty_positions;
    
    for(int i = 0; i < bsize; i++ )
    {
        for(int j = 0; j < bsize; j++) {
            if(get_node_value(i, j) == Hex_Color::EMPTY) {
                empty_positions.push_back(make_pair(i*bsize+j,0));
            }
        }
    }
    
    // Iterate Monte Carlo simulation for the vector
    for(int i = 0;i < empty_positions.size();i++) {
        vector<int> random_sequence(empty_positions.size()-1);
        
        // skip current position (i-th element of empty_positions)
        for(int j = 0; j < i ; j++) random_sequence[j] = empty_positions[j].first;
        for(int j = i+1; j < empty_positions.size() ; j++) random_sequence[j-1] = empty_positions[j].first;

        // Monte Carlo Simulations in parallel
        vector<future<int>> results;
        vector<packaged_task<int(HEX_Graph*, vector<int> , int, int)>> tasks;
        vector<thread> threads;
        
        for(int num_p = 0; num_p < NUM_THREADS; ++num_p)
            tasks.push_back(packaged_task<int(HEX_Graph*, vector<int>, int, int)>(montecarlo));
      
        for(auto& t:tasks) {
            results.push_back(future<int>(t.get_future()));
            threads.push_back(thread(std::move(t), this, random_sequence, empty_positions[i].first, MC_SIZE/NUM_THREADS));
        }
        for(auto& t: threads) t.join();
        for(auto& r: results) empty_positions[i].second += r.get();
    }
    
    // Sort the vector according to descending winning ratio.
    sort(empty_positions.begin(), empty_positions.end(), [](pair<int,double> a, pair<int,double> b) {return a.second > b.second;});
    
    // set top position as the next move.
    gnode[empty_positions[0].first] = hc;
    
    auto end_time = chrono::steady_clock::now();
    auto c_end = clock();
    auto diff = end_time - start_time;
    
    cout << "  Wall clock time used:\t\t" << setw(10) << right << fixed << showpoint << setprecision(1) << chrono::duration <double, milli> (diff).count() << " msec (user waiting time)" << endl;
    auto time_elapsed_ms = 1000.0 * (c_end-c_start) / CLOCKS_PER_SEC;
    
    cout << "  Accumulated CPU time used:" << setw(10) << right << fixed << showpoint << setprecision(1) << time_elapsed_ms << " msec (" << NUM_THREADS << " threads)" << endl;
}

bool HEX_Graph::get_user_input(const Hex_Color &hc)
{
    int x, y;
    string str_x, str_y;
    
    cout << "Remember that \'X\' should connect horizontally and \'O\' should connect vertically!\n";
    
    do {
        cout << "Input the next move(row, column): ";
        cin >> str_x >> str_y;
        
        try {
            x = stoi(str_x);
            y = stoi(str_y);
            
            if(x < bsize && x >= 0 && y >= 0 && y < bsize) {
                switch (get_node_value(x, y)) {
                    case Hex_Color::BLUE:
                        cout << "\nInvalid move! The position already occupied by \'X\'! Give another position.\n";
                        break;
                    case Hex_Color::RED:
                        cout << "\nInvalid move! The position already occupied by \'O\'! Give another position.\n";
                        break;
                    default:
                        cout << "\nYour input is (" << x <<", " << y <<").\n";
                        set_node_value(x, y, hc);
                        return true;
                }
            }
            else
                cout << "Invalid move! The position is out of range. (use between 0 and " << bsize - 1 << "). Give another position.\n";
        }
        catch(invalid_argument const& ex)
        {
            cout << "input is invalid, try again!\n";
            continue;
        }
        catch(out_of_range const& ex)
        {
            cout << "input is out of range(int), try again!\n";
            continue;
        }
    } while (true);
}

bool PriorityQueue::contains(int node)//: does the queue contain queue_element.
{
    for(auto elm : c)
    {
        if (elm.first == node ) return true;
    }
    return false;
}

long PriorityQueue::getPriority(int node)//: changes the priority (node value) of queue element.
{
    for(auto elm : c) {
        if (elm.first == node) return elm.second;
    }
    return -1;
}

bool sortbysecdesc(const pair<int,long> &a,
              const pair<int,long> &b)
{
    return (a.second > b.second);
}

bool PriorityQueue::chgPriority(int node, long priority)//: changes the priority (node value) of queue element.
{
    for(int i = 0; i < c.size(); i++)
    {
        if (c[i].first == node && c[i].second > priority)
        {
            c[i].second = priority;

            sort(c.begin(),c.end(), sortbysecdesc);  // sort descending order of second element.
            
            return true;
        }
    }
    return false;
}

