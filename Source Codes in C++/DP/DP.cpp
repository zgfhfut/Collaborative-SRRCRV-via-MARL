// DP.cpp
#include <stack>
#include <thread>
#include <mutex>
#include <windows.h>
#include <sstream>
#include <iomanip>
#include<iostream>
#include<ctime>
#include "road.h"
#include "random.h"
/*******************************Dynamic Programming********************************/
using namespace std;

mutex action_mutex; // Operation lock for the action set

mutex road_state_mutex; // Operation lock for the state space

mutex try_mutex;

map<int, bool> node_rescue;
map<double, set<Line>> all_time_state;
map<double, Line> time_state;
map<double, Line> times;
map<double, double> time_state_1; // Delivery time for all demand points
map<double, Line> best_time;
map<double, int> best_time_r;
map<int, int> best_agent;

map<int, int> allagent;

set<int> reserve_node_store; // Store reserve points

set<int> node_store; // Store road network nodes

set<int> const_demand_node_store; // Store the original road network demand points before transformation,
set<int> const_demand_node_store1; // Store the original road network demand points before transformation, used for initialization

set<int> demand_node_store; // Damaged demand points
//set<int> demand_node_store1; // Demand points for material distribution

set<Line> edge_store; // Store road network edges

set<Line> const_damaged_edge_store; // Store the original road network damaged edges before transformation, used for initialization
set<Line> damaged_edge_store;

map<int, double> max_allowed_dis_store_0; // Store maximum allowed distance
map<int, double> max_allowed_dis_store_1; // Store maximum allowed distance
map<int, double> max_allowed_dis_store_2; // Store maximum allowed distance
map<int, double> max_allowed_dis_store_3; // Store maximum allowed distance
map<int, double> max_allowed_dis_store_4; // Store maximum allowed distance

/***********************************************************************/
/*
The map container is a key-value pair collection. To have one label correspond to one state, the state set is used as the key.
Because the map requires unique keys, using the state set as the key ensures that no duplicate states are inserted during storage.
This makes the program easiest to process, rather than the conventional way (using label as the key).
*/

vector<Line> agent_best_pi_1;
vector<Line> agent_best_pi_2;
vector<Line> agent_best_pi_3;
vector<Line> agent_best_pi_4;
vector<Line> agent_best_pi_5;
vector<int> agent_best_pi_6;
vector<int> agent_best_pi_7;
vector<int> agent_best_pi_8;
vector<int> agent_best_pi_9;
vector<int> agent_best_pi_10;

set<Line> unexe_actions_set; // Record the set of unexecuted actions
set<int> unexe_actions_set1; // Record the set of unexecuted actions

void initialize() {
    srand((unsigned)time(NULL)); // Initialize the random function
}

void Initial_Graph()
{
    demand_node_store.clear();

    agent_best_pi_1.clear();
    agent_best_pi_2.clear();
    agent_best_pi_3.clear();
    agent_best_pi_4.clear();
    agent_best_pi_5.clear();
    agent_best_pi_6.clear();
    agent_best_pi_7.clear();
    agent_best_pi_8.clear();
    agent_best_pi_9.clear();
    agent_best_pi_10.clear();

    for (auto node_iter = const_demand_node_store.begin(); node_iter != const_demand_node_store.end(); ++node_iter)
    {
        demand_node_store.insert(*node_iter);
    }

    damaged_edge_store.clear();
    for (auto edge_iter = const_damaged_edge_store.begin(); edge_iter != const_damaged_edge_store.end(); ++edge_iter)
    {
        damaged_edge_store.insert(*edge_iter);
    }
}

void createGraph(Graph& g, string read_path, int agent_num)
{
    node_store.clear();

    const_demand_node_store.clear();

    edge_store.clear();

    const_damaged_edge_store.clear();

    max_allowed_dis_store_0.clear();

    max_allowed_dis_store_1.clear();

    max_allowed_dis_store_2.clear();

    max_allowed_dis_store_3.clear();

    max_allowed_dis_store_4.clear();

    /**********************g.addEdge**********************/
    string add_edge = read_path + "add_edge.txt";
    ifstream infile1;
    infile1.open(add_edge);
    while (!infile1.eof())
    {
        int src, dest;
        double weight;
        string oneline;

        getline(infile1, oneline);
        stringstream ssin(oneline);

        ssin >> src;
        ssin >> dest;
        ssin >> weight;

        g.addEdge(src, dest, weight);

        edge_store.insert(Line(src, dest));
    }
    infile1.close();

    /**********************g.setImportance**********************/
    string set_node = read_path + "set_node.txt";
    ifstream infile2;
    infile2.open(set_node);
    while (!infile2.eof())
    {
        int node;
        double importance;
        string oneline;

        getline(infile2, oneline);
        stringstream ssin(oneline);

        ssin >> node;
        ssin >> importance;

        g.setImportance(node, importance);

        node_store.insert(node);

        if (importance != 0) const_demand_node_store.insert(node);
    }
    infile2.close();

    /**********************g.repairTime**********************/
    string set_repair = read_path + "set_repair.txt";
    ifstream infile3;
    infile3.open(set_repair);
    while (!infile3.eof())
    {
        int src, dest;
        double repair_time;
        string oneline;

        getline(infile3, oneline);
        stringstream ssin(oneline);

        ssin >> src;
        ssin >> dest;
        ssin >> repair_time;

        g.setRepairTime(Line(src, dest), repair_time);

        const_damaged_edge_store.insert(Line(src, dest));
    }
    infile3.close();

    /**********************MAX_DISTANCE**********************/
    for (int index = 0; index < agent_num; ++index)
    {
        string max_dis = read_path + "to_" + to_string(index) + "_max_dis.txt";
        ifstream infile4;
        infile4.open(max_dis);
        while (!infile4.eof())
        {
            int node;
            double max_allowed_dis;
            string oneline;

            getline(infile4, oneline);
            stringstream ssin(oneline);

            ssin >> node;
            ssin >> max_allowed_dis;

            switch (index)
            {
            case 0:
                max_allowed_dis_store_0.insert(make_pair(node, max_allowed_dis));
                break;
            case 1:
                max_allowed_dis_store_1.insert(make_pair(node, max_allowed_dis));
                break;
            case 2:
                max_allowed_dis_store_2.insert(make_pair(node, max_allowed_dis));
                break;
            case 3:
                max_allowed_dis_store_3.insert(make_pair(node, max_allowed_dis));
                break;
            case 4:
                max_allowed_dis_store_4.insert(make_pair(node, max_allowed_dis));
                break;
            default:
                break;
            }
        }
        infile4.close();

        reserve_node_store.insert(index);
        g.setImportance(index, 0); // Set the importance of reserve points
    }

    /**********************RESCUE**********************/
    string set_rescue = read_path + "set_rescue.txt";
    ifstream infile5;
    infile5.open(set_rescue);
    while (!infile5.eof()) {
        int node;
        double rescue_time;
        string oneline;

        getline(infile5, oneline);
        stringstream ssin(oneline);

        ssin >> node;
        ssin >> rescue_time;

        g.setRescueTime(node, rescue_time); // Time spent delivering supplies at the demand point

        //const_demand_node_store1.insert(node);

        //node_rescue.insert(make_pair(node, false));
    }
    infile5.close();

    for (auto iter = const_damaged_edge_store.begin(); iter != const_damaged_edge_store.end(); ++iter)
    {
        g.setConnectionStatus(*iter, false);
    }
}


void Initial_State(Graph g, set<int>& demand_nodes_set, int last_index)
{
    vector<double> dis_to;

    g.shortestDis(last_index, dis_to); // Find the shortest distance from all nodes to point 0, store it in dis_to_0
    map<int, double> max_allowed_dis_store;
    switch (last_index)
    {
    case 0:
        max_allowed_dis_store = max_allowed_dis_store_0;
        break;
    case 1:
        max_allowed_dis_store = max_allowed_dis_store_1;
        break;
    case 2:
        max_allowed_dis_store = max_allowed_dis_store_2;
        break;
    case 3:
        max_allowed_dis_store = max_allowed_dis_store_3;
        break;
    case 4:
        max_allowed_dis_store = max_allowed_dis_store_4;
        break;
    default:
        break;
    }
    for (auto node_iter = demand_node_store.begin(); node_iter != demand_node_store.end(); ++node_iter) // Simplify the action set
    {
        if (dis_to[*node_iter] <= max_allowed_dis_store[*node_iter])
        {
            demand_nodes_set.erase(*node_iter);
        }
    }
    //cout << "unexe_actions_set.size() = " << unexe_actions_set.size() << endl;
    //cout << "state_set.size() = " << state_set.size() << endl;
}
/* Enter road network change information here */

/*
To get a random integer in [a, b), use (rand() % (b - a)) + a;
Another representation: a + (int)(b-a) * rand() / (RAND_MAX + 1);
To get a random integer in [a, b], use (rand() % (b - a + 1)) + a;
Another representation: a + (int)((b - a)+1) * rand() / (RAND_MAX + 1);
To get a random integer in (a, b], use (rand() % (b - a)) + a + 1;
General formula: a + rand() % n; where a is the start value and n is the integer range;
To get a float in [0,1), use rand() / double(RAND_MAX);
To get a float in [0,1], use rand() / double(RAND_MAX+1);
To get a float in [0~10], use rand() /(double)(RAND_MAX/10);
To get a float in [0~100], use rand() /(double)(RAND_MAX/100);
*/

// Generate a random integer between [a,b)
int getRandom(int lowerRound, int upperRound)
{
    int value;

    value = lowerRound + (int)(upperRound - lowerRound) * rand() / (RAND_MAX + 1);

    return value;
}

// Generate a float between (0,1)
double getDoubleRandom()
{
    double double_rand;

    double_rand = rand() / double((double)RAND_MAX + 1);
    //double_rand = randomperca(); // Use the normal distribution function to get a (0,1) random value, more random than rand()
    //Original method: double_rand = rand() / double(RAND_MAX + 1);
    return double_rand;
}

// String split function, can split string s according to the given delimiter delim
// Use "-" as the delimiter to separate "state_label" and "action" from "state_label-action"
void mySplitString(string s, string delim, vector<string>& ret)
{
    string::size_type split_pos = 0; // Record the starting position of each split

    string::size_type index = s.find_first_of(delim, split_pos);
    /*
    s.find_first_of(arg1,arg2) finds the first occurrence of arg1 in s starting from index arg2, returns its position index. If not found, returns -1;
    */

    while (index != -1)
    {
        ret.push_back(s.substr(split_pos, index - split_pos)); // Store the characters between the split position and the delimiter position into ret
        //|state_label|-action, store state_label into ret
        //s.substr(arg1,arg2) string截断 function: start from arg1,截取 arg2 characters

        split_pos = index + 1; // Record the starting position for the next split
        index = s.find_first_of(delim, split_pos);
    }

    // If there are elements after the last delimiter
    if (s.length() - split_pos > 0)
    {
        ret.push_back(s.substr(split_pos));
    }
}

void accessbiltyCheck(Graph g, Line& repair_action, set<int> demand_set, set<int>& demand_available_nodes) {
    vector<vector<int>> all_path_store;
    vector<double> dist_store;

    // dijkstra
    g.shortestPath(0, dist_store, all_path_store);

    for (set<int>::iterator demand_iter = demand_set.begin(); demand_iter != demand_set.end(); ++demand_iter) {
        for (vector<int>::iterator path_iter = all_path_store[*demand_iter].begin(); path_iter != all_path_store[*demand_iter].end(); ++path_iter) {
            if (path_iter + 1 != all_path_store[*demand_iter].end()) {
                if ((*path_iter == repair_action.src && *(path_iter + 1) == repair_action.dest) || (*path_iter == repair_action.dest && *(path_iter + 1) == repair_action.src)) {
                    if (dist_store[*demand_iter] < max_allowed_dis_store_0[*demand_iter]) {
                        demand_available_nodes.insert(*demand_iter);
                    }
                }
            }
        }
    }// return demand_available_nodes
}

double ExeAct(Graph& g, Line action_line, int agent_num,
    map<int, vector<double>>& dis_pre, map<int, vector<double>>& dis_aft)
{
    for (int i = 0; i < agent_num; ++i)
    {
        vector<double> dis_array;
        g.shortestDis(i, dis_array);
        dis_pre[i] = dis_array;
    }

    double edge_weight = g.setConnectionStatus(action_line, true);

    for (int i = 0; i < agent_num; ++i)
    {
        vector<double> dis_array;
        g.shortestDis(i, dis_array);
        dis_aft[i] = dis_array;
    }

    return edge_weight; // Return the edge weight
}

double CalReward1(Graph g, int rescue_dest, double dis)
{
    double reward = 0;
    double lambda = 0.1;
    bool re_flag = false;

    //cout << rescue_src << " " << rescue_dest << ":" << dis;
    reward = g.getImportance(rescue_dest);
    //reward = getRandom(0, 10);
    //cout <<"reward = " << dis <<" "<< reward << endl;
    return reward;
}

double CalReward(Graph g, int agent_num, double last_total_cost, double total_cost,
    map<int, vector<double>> dis_pre, map<int, vector<double>> dis_aft)
{
    double reward = 0;
    double lambda = 0.1;
    bool re_flag = false;
    set<int> demand_available_nodes;
    for (auto node_iter = node_store.begin(); node_iter != node_store.end(); ++node_iter)
    {
        if (dis_aft[0][*node_iter] <= max_allowed_dis_store_0[*node_iter]
            && dis_pre[0][*node_iter] > max_allowed_dis_store_0[*node_iter])
        {
            demand_available_nodes.insert(*node_iter);
            //reward += (lambda * g.getImportance(*node_iter) * 100 / dis_aft[0][*node_iter]) + ((1 - lambda) * g.getImportance(*node_iter) * 100 / total_cost);
            re_flag = true;
        }
        if (dis_aft[1][*node_iter] <= max_allowed_dis_store_1[*node_iter]
            && dis_pre[1][*node_iter] > max_allowed_dis_store_1[*node_iter])
        {
            demand_available_nodes.insert(*node_iter);
            //reward += (lambda * g.getImportance(*node_iter) * 100 / dis_aft[1][*node_iter]) + ((1 - lambda) * g.getImportance(*node_iter) * 100 / total_cost);
            re_flag = true;
        }
        if (dis_aft[2][*node_iter] <= max_allowed_dis_store_2[*node_iter]
            && dis_pre[2][*node_iter] > max_allowed_dis_store_2[*node_iter])
        {
            demand_available_nodes.insert(*node_iter);
            //reward += (lambda * g.getImportance(*node_iter) * 100 / dis_aft[2][*node_iter]) + ((1 - lambda) * g.getImportance(*node_iter) * 100 / total_cost);
            re_flag = true;
        }
        if (agent_num == 5 && dis_aft[3][*node_iter] <= max_allowed_dis_store_3[*node_iter]
            && dis_pre[3][*node_iter] > max_allowed_dis_store_3[*node_iter])
        {
            demand_available_nodes.insert(*node_iter);
            //reward += (lambda * g.getImportance(*node_iter) * 100 / dis_aft[3][*node_iter]) + ((1 - lambda) * g.getImportance(*node_iter) * 100 / total_cost);
            re_flag = true;
        }
        if (agent_num == 5 && dis_aft[4][*node_iter] <= max_allowed_dis_store_4[*node_iter]
            && dis_pre[4][*node_iter] > max_allowed_dis_store_4[*node_iter])
        {
            demand_available_nodes.insert(*node_iter);
            //reward += (lambda * g.getImportance(*node_iter) * 100 / dis_aft[4][*node_iter]) + ((1 - lambda) * g.getImportance(*node_iter) * 100 / total_cost);
            re_flag = true;
        }
    }

    for (set<int>::iterator iter = demand_available_nodes.begin(); iter != demand_available_nodes.end(); ++iter) {
        reward += g.getImportance(*iter) / total_cost;
    }
    if (!re_flag)
    {
        reward = 1 / total_cost;
    }
    //reward = getRandom(0, 10);
    //cout << "reward = "<< reward << endl;
    return reward;
}

// find min cost and repair action
// Simulate + select action
bool findMinRepairCost(Graph g, int agent_num, set<Line> action_space, set<int> demand_set, Line& repair_action, int& next_dest, set<int>& demand_available_nodes, double& time, set<Line> V_ACT) {
    double last_total_cost = time;
    double min_repair_cost;
    double temp_repair_cost;

    /*temp_cost_store.clear();
    temp_demand_available_store.clear();
    */
    // temp_demand_available_nodes needs to be cleared before the loop ends
    double temp_min_cost = 0, max_reward = 0;
    Line temp_repair_action(0, 0);
    map<int, vector<double>> dis_pre, dis_aft;
    vector<double> temp_travel_dis;
    g.shortestDis(next_dest, temp_travel_dis);
    set<Line> actions_set;
    actions_set.clear();
    //cout << "action_space.size() = " << action_space.size() << endl;
    for (set<Line>::iterator iter = action_space.begin(); iter != action_space.end(); ++iter) {
        if ((temp_travel_dis[iter->src] < INF || temp_travel_dis[iter->dest] < INF) && V_ACT.find(*iter) == V_ACT.end())
            actions_set.insert(*iter);
    }
    if (actions_set.size() == 0) {
        time += 5;
        cout << "error!" << endl;
        return false;
    }
    for (set<Line>::iterator iter = actions_set.begin(); iter != actions_set.end(); ++iter) {

        temp_repair_action = *iter;
        double repair_travel_cost;
        double repair_line_cost;
        double repair_action_cost;
        dis_pre.clear();
        dis_aft.clear();
        ExeAct(g, temp_repair_action, 5, dis_pre, dis_aft);

        int last_dest = next_dest;
        int next_action_dest;
        //g.shortestDis(last_dest, temp_travel_dis);

        if (temp_travel_dis[iter->src] <= temp_travel_dis[iter->dest]) {
            next_action_dest = iter->dest;
            repair_travel_cost = temp_travel_dis[iter->src];
        }
        else {
            next_action_dest = iter->src;
            repair_travel_cost = temp_travel_dis[iter->dest];
        }

        repair_line_cost = g.getWeight(*iter) + g.getRepairTime(*iter);
        repair_action_cost = repair_travel_cost + repair_line_cost;

        last_dest = next_action_dest;  // !
        int total_cost = last_total_cost + repair_action_cost;
        //cout<< " cost = " << last_total_cost << " " << repair_travel_cost << endl;
        double temp_reward = CalReward(g, agent_num, last_total_cost, total_cost, dis_pre, dis_aft);

        // If temp_repair_cost is 0, it means repairing one node did not connect a new demand node, this case should be removed when comparing repair_cost

        //cout << "repair_action_cost = " << repair_action_cost <<"g.getWeight(*iter) = "<< g.getWeight(*iter) << " " << iter->src << " - " << iter->dest << endl;
        if (temp_reward >= max_reward) {

            temp_min_cost = repair_action_cost;

            max_reward = temp_reward;
            next_dest = last_dest;
            repair_action = *iter;
            //demand_available_nodes = temp_demand_available_nodes;
        }
        // Need to reset the repair_action node status to false
        g.setConnectionStatus(*iter, false);  // !

    }
    //cout << repair_action.src << " - " << repair_action.dest << endl;
    // Find the repair_store corresponding to min_cost, then take out demand_available_nodes
    time = last_total_cost + temp_min_cost;
    return true;
    //cout << "last_total_cost = " << last_total_cost<<" " <<time << endl;
}

bool findMinRepairCost_rescue(Graph g, set<int> action_space, int& repair_action, int& next_dest, set<int>& demand_available_nodes, double last_total_cost, double& time, set<int> V_ACT) {
    last_total_cost = time;
    double min_repair_cost;
    double temp_repair_cost;

    /* The commented code is not necessary, the execution result is the same. Because in the dpMethod function below, each loop starts from the beginning of this function,
    and the following two objects are in the function body, their scope does not exceed the function body.
    temp_cost_store.clear();
    temp_demand_available_store.clear();
    */
    // temp_demand_available_nodes needs to be cleared before the loop ends
    double temp_min_cost = 0, max_reward = 0;
    int temp_repair_action = 0;
    map<int, vector<double>> dis_pre, dis_aft;

    set<int> actions_set;
    actions_set.clear();
    vector<double> temp_travel_dis;
    vector<Line> s;
    for (auto iter = time_state.begin(); iter != time_state.end(); iter++) {
        if (last_total_cost >= iter->first) {
            g.setConnectionStatus(iter->second, true);
            s.push_back(iter->second);
        }
        else break;
    }
    g.shortestDis(next_dest, temp_travel_dis);

    for (auto iter = s.begin(); iter != s.end(); iter++) g.setConnectionStatus(*iter, false);

    for (set<int>::iterator iter = action_space.begin(); iter != action_space.end(); ++iter) {
        if (temp_travel_dis[*iter] < INF && V_ACT.find(*iter) == V_ACT.end()) {
            actions_set.insert(*iter);
        }
    }
    //cout<< " actions_set.size() = "<< actions_set.size() << endl;
    if (actions_set.size() == 0) {
        cout << "wait!!!" << time << endl;
        time += 5;
        return false;
    }
    for (set<int>::iterator iter = actions_set.begin(); iter != actions_set.end(); ++iter) {
        // set repair_liness[i] repair status -> true
        temp_repair_action = *iter;
        double repair_travel_cost;
        double repair_line_cost;
        double repair_action_cost;
        dis_pre.clear();
        dis_aft.clear();

        // accessiblity check
        //accessbiltyCheck(g, temp_repair_action, demand_set, temp_demand_available_nodes);

        repair_action_cost = temp_travel_dis[*iter] + g.getRescueTime(*iter);

        //cout<< " cost = " << last_total_cost << " " << repair_travel_cost << endl;
        // calculate cost
        double temp_reward = CalReward1(g, temp_repair_action, temp_travel_dis[*iter]);
        //cout << "temp_reward = " << temp_reward << endl;
        //double temp_reward = calculateCost(g, repair_action_cost, temp_demand_available_nodes);  // record the cost

        //cout << "repair_action_cost = " << repair_action_cost <<"g.getWeight(*iter) = "<< g.getWeight(*iter) << " " << iter->src << " - " << iter->dest << endl;

        if (temp_reward >= max_reward) {
            temp_min_cost = repair_action_cost;
            max_reward = temp_reward;
            next_dest = *iter;
            repair_action = *iter;
        }
    }
    time = last_total_cost + temp_min_cost;
    //cout << "last_total_cost = " << last_total_cost << " " << time << endl;

    //return min_repair_cost, repair_action, demand_available_nodes
    return true;
}

void Bubble1(vector<int>& V_ACT, map<int, double>& action_fine)
{
    int flag;
    int temp = 0;
    int n = V_ACT.size();

    for (int p = n - 1; p >= 0; --p)
    {
        flag = 0;
        for (int index = 0; index < p; ++index)
        {
            if (action_fine[V_ACT[index]] < action_fine[V_ACT[index + 1]])
            {
                temp = V_ACT[index];
                V_ACT[index] = V_ACT[index + 1];
                V_ACT[index + 1] = temp;
                flag = 1;
            }
        }
        if (flag == 0)
            break;
    }
}

// Bubble sort [descending order]
void Bubble(vector<Line>& V_ACT, map<Line, double>& action_fine)
{
    int flag;
    Line temp(0, 0);
    int n = V_ACT.size();

    for (int p = n - 1; p >= 0; --p)
    {
        flag = 0;
        for (int index = 0; index < p; ++index)
        {
            if (action_fine[V_ACT[index]] < action_fine[V_ACT[index + 1]])
            {
                temp = V_ACT[index];
                V_ACT[index] = V_ACT[index + 1];
                V_ACT[index + 1] = temp;
                flag = 1;
            }
        }
        if (flag == 0)
            break;
    }
}

bool test(Graph g) {
    for (set<Line>::iterator damaged_iter = damaged_edge_store.begin(); damaged_iter != damaged_edge_store.end(); ++damaged_iter) {
        g.setConnectionStatus(*damaged_iter, false);
    }
    for (auto iter = time_state.begin(); iter != time_state.end(); ++iter) g.setConnectionStatus(iter->second, true);

    set<int> rest_demand_nodes = demand_node_store;   // Initialize remaining demand nodes
    for (int index = 0; index < 5; ++index)
    {
        vector<double> dis_array;
        g.shortestDis(index, dis_array);
        map<int, double> max_allowed_dis_store;
        switch (index) {
        case 0:max_allowed_dis_store = max_allowed_dis_store_0; break;
        case 1:max_allowed_dis_store = max_allowed_dis_store_1; break;
        case 2:max_allowed_dis_store = max_allowed_dis_store_2; break;
        case 3:max_allowed_dis_store = max_allowed_dis_store_3; break;
        case 5:max_allowed_dis_store = max_allowed_dis_store_4; break;
        default:break;
        }
        for (auto node_iter = demand_node_store.begin(); node_iter != demand_node_store.end(); ++node_iter)
        {
            if (dis_array[*node_iter] < INF)
                rest_demand_nodes.erase(*node_iter);
        }
    }
    if (rest_demand_nodes.size() != 0) {
        cout << "rest_demand_nodes.size() = " << rest_demand_nodes.size() << " time_state.size() = " << time_state.size() << endl;
        return false;
    }
    return true;
}

void dpMethod(Graph g, int agent_num, double lambda, double& object_func, double& object_func_rescue) {

    for (set<Line>::iterator damaged_iter = damaged_edge_store.begin(); damaged_iter != damaged_edge_store.end(); ++damaged_iter)
        g.setConnectionStatus(*damaged_iter, false);

    for (set<int>::iterator damaged_iter = demand_node_store.begin(); damaged_iter != demand_node_store.end(); ++damaged_iter)
        node_rescue[*damaged_iter] = false;

    set<Line> action_space = damaged_edge_store; // Container initialization method (C++11 standard)
    set<int> rest_demand_nodes = demand_node_store;
    unexe_actions_set1 = demand_node_store;
    double weighted_repair_cost = 0;
    double weighted_dis_cost = 0;
    set<int> demand_available_nodes;

    time_state.clear();

    double time_1 = 0, time_2 = 0, time_3 = 0, time = 0, time_4 = 0, time_5 = 0, time_6 = 0, time_7 = 0, time_8 = 0, time_9 = 0, time_10 = 0;

    bool exe_flag_1 = false, exe_flag_2 = false, exe_flag_3 = false, exe_flag_4 = false, exe_flag_5 = false; // Flag indicating whether the previous action was executed

    bool exe_flag_6 = false, exe_flag_7 = false, exe_flag_8 = false, exe_flag_9 = false, exe_flag_10 = false; // Flag indicating whether the previous action was executed

    bool act_flag_1 = false, act_flag_2 = false, act_flag_3 = false, act_flag_4 = false, act_flag_5 = false;

    bool act_flag_6 = false, act_flag_7 = false, act_flag_8 = false, act_flag_9 = false, act_flag_10 = false; // Flag indicating whether a new action needs to be pushed onto the stack

    double total_cost_1 = 0, total_cost_2 = 0, total_cost_3 = 0, total_cost_4 = 0, total_cost_5 = 0;

    double total_cost_6 = 0, total_cost_7 = 0, total_cost_8 = 0, total_cost_9 = 0, total_cost_10 = 0;

    int last_dest_1 = 0, last_dest_2 = 1, last_dest_3 = 2, last_dest_4 = 3, last_dest_5 = 4, last_dest_6 = 0, last_dest_7 = 1, last_dest_8 = 2, last_dest_9 = 3, last_dest_10 = 4;

    Line act_1(0, 0), act_2(1, 1), act_3(2, 2), act_4(3, 3), act_5(4, 4);

    int act_6 = 0, act_7 = 1, act_8 = 2, act_9 = 3, act_10 = 4;

    vector<Line> V_ACT; // Buffer storing actions to be executed
    map<Line, int> action_agent; // Record the repair team corresponding to the action
    map<Line, double> action_fine; // Record the execution time of the corresponding action
    Line exe_action(0, 0);

    // First, initialize, remove already connected nodes from the graph
    vector<vector<int>> temp_all_path_store;

    vector<double> dis_to_0;
    vector<double> dis_to_1;
    vector<double> dis_to_2;
    vector<double> dis_to_3;
    vector<double> dis_to_4;

    for (int i = 0; i < agent_num; i++) {
        switch (i)
        {
        case 0: {
            Initial_State(g, rest_demand_nodes, 0);
            exe_flag_1 = true;
            exe_flag_6 = true;
            break;
        }
        case 1: {
            Initial_State(g, rest_demand_nodes, 1);
            exe_flag_2 = true;
            exe_flag_7 = true;
            break;
        }
        case 2: {
            Initial_State(g, rest_demand_nodes, 2);
            exe_flag_3 = true;
            exe_flag_8 = true;
            break;
        }
        case 3: {
            Initial_State(g, rest_demand_nodes, 3);
            exe_flag_4 = true;
            exe_flag_9 = true;
            break;
        }
        case 4: {
            Initial_State(g, rest_demand_nodes, 4);
            exe_flag_5 = true;
            exe_flag_10 = true;
            break;
        }
        default:
            break;
        }
    }

    cout << "rest_demand_nodes1 = " << rest_demand_nodes.size() << endl;
    double object_func_rescue1 = 0;
    // Dynamic programming, solve the problem from the bottom up, starting from the smallest subproblem, weighted_repair_cost stores the value of the optimal subproblem, which will be used in the next step of calculation
    set<Line> v;
    while (!rest_demand_nodes.empty()) {

        act_flag_1 = act_flag_2 = act_flag_3 = act_flag_4 = act_flag_5 = false;

        if (exe_flag_1)
        {
            findMinRepairCost(g, agent_num, action_space, rest_demand_nodes, act_1, last_dest_1, demand_available_nodes, time_1, v); // demand_available_nodes are the demand nodes connected this time
            v.insert(act_1);
            //cout << "1111111111111111--" << act_1.src <<"-"<< act_1.dest<<" "<< time_1 << endl;
            action_agent[act_1] = 1; // Set the repair team executing action act_1

            action_fine[act_1] = time_1; // Store the time after executing act_1
            exe_flag_1 = false;
            act_flag_1 = true;
        }
        if (exe_flag_2)
        {
            findMinRepairCost(g, agent_num, action_space, rest_demand_nodes, act_2, last_dest_2, demand_available_nodes, time_2, v); // demand_available_nodes are the demand nodes connected this time
            v.insert(act_2);
            action_agent[act_2] = 2;
            //cout << "222222222222222222--" << act_2.src << "-" << act_2.dest << " " << time_2 << endl;
            action_fine[act_2] = time_2;
            exe_flag_2 = false;
            act_flag_2 = true;
        }
        if (exe_flag_3)
        {
            findMinRepairCost(g, agent_num, action_space, rest_demand_nodes, act_3, last_dest_3, demand_available_nodes, time_3, v); // demand_available_nodes are the demand nodes connected this time
            v.insert(act_3);
            action_agent[act_3] = 3;
            //cout << "3333333333333333--" << act_3.src << "-" << act_3.dest << " " << time_3 << endl;
            action_fine[act_3] = time_3;
            exe_flag_3 = false;
            act_flag_3 = true;
        }
        if (exe_flag_4)
        {
            findMinRepairCost(g, agent_num, action_space, rest_demand_nodes, act_4, last_dest_4, demand_available_nodes, time_4, v); // demand_available_nodes are the demand nodes connected this time
            v.insert(act_4);
            action_agent[act_4] = 4;
            action_fine[act_4] = time_4;
            exe_flag_4 = false;
            act_flag_4 = true;
        }
        if (exe_flag_5)
        {
            findMinRepairCost(g, agent_num, action_space, rest_demand_nodes, act_5, last_dest_5, demand_available_nodes, time_5, v); // demand_available_nodes are the demand nodes connected this time
            v.insert(act_5);
            action_agent[act_5] = 5;
            //cout << "5555555555555555--" << act_5.src << "-" << act_5.dest << " " << time_5 << endl;
            action_fine[act_5] = time_5;
            exe_flag_5 = false;
            act_flag_5 = true;
        }

        if (act_flag_1)
            V_ACT.push_back(act_1); // V_ACT.size()=333.......321

        if (act_flag_2)
            V_ACT.push_back(act_2);

        if (act_flag_3)
            V_ACT.push_back(act_3);

        if (act_flag_4)
            V_ACT.push_back(act_4);

        if (act_flag_5)
            V_ACT.push_back(act_5);
        //cout << " V_ACT.size() = " << V_ACT.size() << endl;
        /************************************************************************/

        if (V_ACT.size() > 1)
        {
            Bubble(V_ACT, action_fine);
            int min_index = V_ACT.size(); // Index of the action with the smallest time consumption
            exe_action = V_ACT[min_index - 1]; // The action with the smallest time consumption currently, take one action each time
            auto it = V_ACT.begin() + min_index - 1;
            V_ACT.erase(it);
        }
        else if (V_ACT.size() == 1)
        {
            exe_action = V_ACT[0];
            V_ACT.clear();
        }
        g.setConnectionStatus(exe_action, true);
        time_state.insert(make_pair(action_fine[exe_action], exe_action));
        action_space.erase(exe_action);
        //cout << exe_action.src << "-" << exe_action.dest <<"  "<< action_fine[exe_action] << endl;
        //cout << "action_agent[exe_action] = " <<action_agent[exe_action] << endl;
        switch (action_agent[exe_action])
        {
        case 1: {
            agent_best_pi_1.push_back(exe_action);
            total_cost_1 = time_1;
            exe_flag_1 = true;
            break;
        }
        case 2: {
            agent_best_pi_2.push_back(exe_action);
            total_cost_2 = time_2;
            exe_flag_2 = true;
            break;
        }
        case 3: {
            agent_best_pi_3.push_back(exe_action);
            total_cost_3 = time_3;
            exe_flag_3 = true;
            break;
        }
        case 4: {
            agent_best_pi_4.push_back(exe_action);
            total_cost_4 = time_4;
            exe_flag_4 = true;
            break;
        }
        case 5: {
            agent_best_pi_5.push_back(exe_action);
            total_cost_5 = time_5;
            exe_flag_5 = true;
            break;
        }
        default:break;
        }

        dis_to_0.clear();
        dis_to_1.clear();
        dis_to_2.clear();
        dis_to_3.clear();
        dis_to_4.clear();

        g.shortestDis(0, dis_to_0);
        g.shortestDis(1, dis_to_1);
        g.shortestDis(2, dis_to_2);
        g.shortestDis(3, dis_to_3);
        g.shortestDis(4, dis_to_4);

        set<int> tmp_node_store = rest_demand_nodes;
        for (auto node_iter = tmp_node_store.begin(); node_iter != tmp_node_store.end(); ++node_iter)
        {
            if (dis_to_0[*node_iter] <= max_allowed_dis_store_0[*node_iter]
                || dis_to_1[*node_iter] <= max_allowed_dis_store_1[*node_iter]
                || dis_to_2[*node_iter] <= max_allowed_dis_store_2[*node_iter]
                || dis_to_3[*node_iter] <= max_allowed_dis_store_3[*node_iter]
                || dis_to_4[*node_iter] <= max_allowed_dis_store_4[*node_iter]
                )
            {
                weighted_repair_cost += g.getImportance(*node_iter) * action_fine[exe_action];
                //cout << weighted_repair_cost << endl;
                rest_demand_nodes.erase(*node_iter);
                //cout << " Connected disaster point: " << *node_iter << "   weighted_repair_cost = " << weighted_repair_cost << endl;
            }
        }
        if (rest_demand_nodes.empty()) {
            cout << "Repair finished!" << time_state.size() << endl;
            exe_flag_1 = exe_flag_2 = exe_flag_3 = exe_flag_4 = exe_flag_5 = false;
        }
        //cout <<"rest_demand_nodes.size = "<< rest_demand_nodes.size() << endl;
    }
    dis_to_0.clear();
    dis_to_1.clear();
    dis_to_2.clear();
    dis_to_3.clear();
    dis_to_4.clear();

    g.shortestDis(0, dis_to_0); // Find the shortest distance from all nodes to point 0, store in dis_to_0
    g.shortestDis(1, dis_to_1); // Find the shortest distance from all nodes to point 1, store in dis_to_1
    g.shortestDis(2, dis_to_2); // Find the shortest distance from all nodes to point 2, store in dis_to_2
    g.shortestDis(3, dis_to_3); // Find the shortest distance from all nodes to point 3, store in dis_to_3
    g.shortestDis(4, dis_to_4); // Find the shortest distance from all nodes to point 4, store in dis_to_4

    for (auto node_iter = demand_node_store.begin(); node_iter != demand_node_store.end(); ++node_iter)
    {
        double min_dis = INF;
        for (int index = 0; index < agent_num; ++index)
        {
            vector<double> dis_array;
            switch (index)
            {
            case 0:dis_array = dis_to_0; break;
            case 1:dis_array = dis_to_1; break;
            case 2:dis_array = dis_to_2; break;
            case 3:dis_array = dis_to_3; break;
            case 4:dis_array = dis_to_4; break;
            default:break;
            }
            if (dis_array[*node_iter] < min_dis)
            {
                min_dis = dis_array[*node_iter];
            }
        }
        weighted_dis_cost += g.getImportance(*node_iter) * min_dis;
    }
    object_func = weighted_dis_cost * lambda + weighted_repair_cost * (1 - lambda);
    cout << " best_obj = " << weighted_dis_cost << endl;

    if (!test(g)) {
        object_func_rescue = INF;
        cout << "Repair strategy is incorrect!" << endl;
    }
    for (set<Line>::iterator damaged_iter = damaged_edge_store.begin(); damaged_iter != damaged_edge_store.end(); ++damaged_iter)
        g.setConnectionStatus(*damaged_iter, false);
    int i = 1;
    vector<int> V_ACT_r;
    map<int, int> action_agent_r; // Record the rescue team corresponding to the action
    map<int, double> action_fine_r; // Record the execution time of the corresponding action
    set<int> v_r;
    time_state_1.clear();
    while (!unexe_actions_set1.empty()) {
        int exe_act = 0; // Record the action to be executed in the buffer
        act_flag_6 = act_flag_7 = act_flag_8 = act_flag_9 = act_flag_10 = false;
        bool quit_flag = false;

        if (exe_flag_6)
        {
            quit_flag = findMinRepairCost_rescue(g, unexe_actions_set1, act_6, last_dest_6, demand_available_nodes, total_cost_6, time_6, v_r); // demand_available_nodes are the demand nodes connected this time
            if (quit_flag) {
                v_r.insert(act_6);
                action_agent_r[act_6] = 1; // Set the rescue team executing action act_1

                action_fine_r[act_6] = time_6; // Store the time after executing act_1
                exe_flag_6 = false;
                act_flag_6 = true;
            }
        }
        if (exe_flag_7)
        {
            quit_flag = findMinRepairCost_rescue(g, unexe_actions_set1, act_7, last_dest_7, demand_available_nodes, total_cost_7, time_7, v_r); // demand_available_nodes are the demand nodes connected this time
            if (quit_flag) {
                v_r.insert(act_7);
                action_agent_r[act_7] = 2; // Set the rescue team executing action act_1 = 1

                action_fine_r[act_7] = time_7; // Store the time after executing act_1
                exe_flag_7 = false;
                act_flag_7 = true;
            }
        }
        if (exe_flag_8)
        {
            quit_flag = findMinRepairCost_rescue(g, unexe_actions_set1, act_8, last_dest_8, demand_available_nodes, total_cost_8, time_8, v_r); // demand_available_nodes are the demand nodes connected this time
            if (quit_flag) {
                v_r.insert(act_8);
                action_agent_r[act_8] = 3; // Set the rescue team executing action act_1 = 1

                action_fine_r[act_8] = time_8; // Store the time after executing act_1
                exe_flag_8 = false;
                act_flag_8 = true;
            }
        }
        if (exe_flag_9)
        {
            quit_flag = findMinRepairCost_rescue(g, unexe_actions_set1, act_9, last_dest_9, demand_available_nodes, total_cost_9, time_9, v_r); // demand_available_nodes are the demand nodes connected this time
            if (quit_flag) {
                v_r.insert(act_9);
                action_agent_r[act_9] = 4; // Set the rescue team executing action act_1 = 1

                action_fine_r[act_9] = time_9; // Store the time after executing act_1
                exe_flag_9 = false;
                act_flag_9 = true;
            }
        }
        if (exe_flag_10)
        {
            quit_flag = findMinRepairCost_rescue(g, unexe_actions_set1, act_10, last_dest_10, demand_available_nodes, total_cost_10, time_10, v_r); // demand_available_nodes are the demand nodes connected this time
            if (quit_flag) {
                v_r.insert(act_10);
                action_agent_r[act_10] = 5; // Set the rescue team executing action act_1 = 1

                action_fine_r[act_10] = time_10; // Store the time after executing act_1
                exe_flag_10 = false;
                act_flag_10 = true;
            }
        }
        // Select the action to execute this time
        if (act_flag_6) V_ACT_r.push_back(act_6); // V_ACT.size()=333.......321

        if (act_flag_7) V_ACT_r.push_back(act_7);

        if (act_flag_8) V_ACT_r.push_back(act_8);

        if (act_flag_9) V_ACT_r.push_back(act_9);

        if (act_flag_10) V_ACT_r.push_back(act_10);

        if (V_ACT_r.size() > 1)
        {
            Bubble1(V_ACT_r, action_fine_r);
            int min_index = V_ACT_r.size(); // Index of the action with the smallest time consumption
            exe_act = V_ACT_r[min_index - 1]; // The action with the smallest time consumption currently, take one action each time
            auto it = V_ACT_r.begin() + min_index - 1;
            V_ACT_r.erase(it);
        }
        else if (V_ACT_r.size() == 1)
        {
            exe_act = V_ACT_r[0];
            V_ACT_r.clear();
        }
        if (node_rescue[exe_act] == false) {
            node_rescue[exe_act] = true;
            unexe_actions_set1.erase(exe_act);
            time_state_1.insert(make_pair(action_fine_r[exe_act], g.getImportance(exe_act)));
            object_func_rescue += action_fine_r[exe_act] * g.getImportance(exe_act);
        }
        cout << "unexe_actions_set1 = " << unexe_actions_set1.size() << " exe_act = " << exe_act << " V_ACT_r.size() = " << V_ACT_r.size() << endl;
        switch (action_agent_r[exe_act])
        {
        case 1: {
            agent_best_pi_6.push_back(exe_act);
            total_cost_6 = time_6;
            exe_flag_6 = true;
            break;
        }
        case 2: {
            agent_best_pi_7.push_back(exe_act);
            total_cost_7 = time_7;
            exe_flag_7 = true;
            break;
        }
        case 3: {
            agent_best_pi_8.push_back(exe_act);
            total_cost_8 = time_8;
            exe_flag_8 = true;
            break;
        }
        case 4: {
            agent_best_pi_9.push_back(exe_act);
            total_cost_9 = time_9;
            exe_flag_9 = true;
            break;
        }
        case 5: {
            agent_best_pi_10.push_back(exe_act);
            total_cost_10 = time_10;
            exe_flag_10 = true;
            break;
        }
        default:break;
        }
        if (unexe_actions_set1.empty()) {
            //cout << "Transport finished!" << endl;
            exe_flag_6 = exe_flag_7 = exe_flag_8 = exe_flag_9 = exe_flag_10 = false;
        }
    }
    cout << "object_func_rescue = " << object_func_rescue << endl;
    cout << "time_state.size() = " << time_state.size() << endl;
    for (auto iter = time_state.begin(); iter != time_state.end(); ++iter)
    {
        cout << iter->first << " ,";
    }
    cout << endl;
    cout << " time_state_1.size() = " << time_state_1.size() << endl;
    for (auto iter = time_state_1.begin(); iter != time_state_1.end(); ++iter)
    {
        cout << iter->first << " ,";
    }
    cout << endl;
}

int main()
{
    vector<int> node_num_store = { 60 };
    vector<int> edge_num_store = { 90 };
    vector<double> demand_pro_store = { 0.6 };
    vector<double> damaged_pro_store = { 0.7 };

    vector<bool> changed_demand_node_flag_store = { false };
    vector<bool> changed_damaged_edge_flag_store = { true };

    map<int, int> AGENT_NUM_store =
    {
        { 40, 3 },
        { 60, 5 },
        { 80, 5 },
    };

    int GRAPH_NUM = 10;

    int CHANGE_GRAPH = 10;

    int TRAIN_TIME = 30;

    int f_clock = 150;

    double v_raito = 0.1;

    double epsilon = 0.9, alpha = 0.4, gamma = 0.2, lambda = 0.1;

    string common_path = "E:\\Test_EXP\\90 Road Network Cases\\";//the path of the road network dataset

    string common_path2 = "E:\\Test_EXP\\DP\\";//Default storage path for experimental results：
	//when you test the algorithm, please use the path suggested on the Githup.
	//For example：copy the whole DP folder to your test path, such as "E:\Test_EXP\"

    for (int i_index = 0; i_index < node_num_store.size(); ++i_index)
    {
        int node_num = node_num_store[i_index];
        int edge_num = edge_num_store[i_index];
        int AGENT_NUM = AGENT_NUM_store[node_num];
        char node[10], edge[10];
        sprintf_s(node, "%d", node_num);
        sprintf_s(edge, "%d", edge_num);
        string nodestr(node);
        string edgestr(edge);
        string node_edge = nodestr + "-" + edgestr + "\\";

        for (int j_index = 0; j_index < demand_pro_store.size(); ++j_index)
        {
            double demand_node_pro = demand_pro_store[j_index];
            double damaged_edge_pro = damaged_pro_store[j_index];

            char de_pro[10], da_pro[10];
            sprintf_s(de_pro, "%.1f", demand_node_pro);
            sprintf_s(da_pro, "%.1f", damaged_edge_pro);
            string deprostr(de_pro);
            string daprostr(da_pro);
            string demand_damage = deprostr + "-" + daprostr + "\\";

            string num_agent = to_string(AGENT_NUM) + "_agent\\";

            for (int graph_index = 1; graph_index <= 10; ++graph_index)
            {
                string connect_path = common_path + node_edge + demand_damage + to_string(graph_index) + "\\";

                //for (int alter_index = 1; alter_index <= 10; ++alter_index)
                //{
                Graph g(node_num, v_raito);

                createGraph(g, connect_path, AGENT_NUM);
                cout << connect_path << endl;

                for (int train_index = 0; train_index < 1; train_index++)
                {
                    Initial_Graph();

                    double best_obj = 0, best_obj_rescue = 0;

                    clock_t start_time = clock();

                    vector<Line> action_result_store;

                    dpMethod(g, AGENT_NUM, lambda, best_obj, best_obj_rescue);

                    clock_t end_time = clock();

                    char str[20];
                    sprintf_s(str, "g%d_c%d_DP", graph_index, AGENT_NUM);
                    string tempstring(str);
                    string filename = tempstring + ".txt";

                    string write_path = common_path2 + node_edge + demand_damage + to_string(AGENT_NUM) + "_agent\\" + "Data\\";

                    ofstream outfile;
                    outfile.open(write_path + filename, ofstream::app);
                    outfile << "DP:" << endl;
                    outfile << "node_num = " << node_num << "edge_num = " << edge_num << ";" << demand_node_pro << "_" << damaged_edge_pro << endl;
                    outfile << "/***********************ql result***********************/" << endl;
                    outfile << "solution:" << endl;
                    outfile << "time_state.size() = " << time_state.size() << endl;
                    for (auto iter = time_state.begin(); iter != time_state.end(); ++iter)
                    {
                        outfile << iter->first << " ";
                    }
                    outfile << endl;
                    outfile << "time_state_1.size() = " << time_state_1.size() << endl;
                    for (auto iter = time_state_1.begin(); iter != time_state_1.end(); ++iter)
                    {
                        outfile << iter->first << " ,";
                    }
                    outfile << endl;
                    outfile << "agent_1 repair_path: ";
                    for (auto iter_1 = agent_best_pi_1.begin(); iter_1 != agent_best_pi_1.end(); ++iter_1)
                    {
                        outfile << "Line(" << iter_1->src << "," << iter_1->dest << ") ";
                        cout << "Line(" << iter_1->src << "," << iter_1->dest << ") ";
                    }
                    cout << endl;
                    outfile << endl;

                    outfile << "agent_2 repair_path: ";
                    for (auto iter_2 = agent_best_pi_2.begin(); iter_2 != agent_best_pi_2.end(); ++iter_2)
                    {
                        outfile << "Line(" << iter_2->src << "," << iter_2->dest << ") ";
                        cout << "Line(" << iter_2->src << "," << iter_2->dest << ") ";
                    }
                    cout << endl;
                    outfile << endl;

                    outfile << "agent_3 repair_path: ";
                    for (auto iter_3 = agent_best_pi_3.begin(); iter_3 != agent_best_pi_3.end(); ++iter_3)
                    {
                        outfile << "Line(" << iter_3->src << "," << iter_3->dest << ") ";
                        cout << "Line(" << iter_3->src << "," << iter_3->dest << ") ";
                    }
                    cout << endl;
                    outfile << endl;

                    outfile << "agent_4 repair_path: ";
                    for (auto iter_2 = agent_best_pi_4.begin(); iter_2 != agent_best_pi_4.end(); ++iter_2)
                    {
                        outfile << "Line(" << iter_2->src << "," << iter_2->dest << ") ";
                        cout << "Line(" << iter_2->src << "," << iter_2->dest << ") ";
                    }
                    cout << endl;
                    outfile << endl;

                    outfile << "agent_5 repair_path: ";
                    for (auto iter_3 = agent_best_pi_5.begin(); iter_3 != agent_best_pi_5.end(); ++iter_3)
                    {
                        outfile << "Line(" << iter_3->src << "," << iter_3->dest << ") ";
                        cout << "Line(" << iter_3->src << "," << iter_3->dest << ") ";
                    }
                    cout << endl;
                    outfile << endl;

                    outfile << "agent_6 repair_path: ";
                    for (auto iter_1 = agent_best_pi_6.begin(); iter_1 != agent_best_pi_6.end(); ++iter_1)
                    {
                        outfile << *iter_1 << "->";
                        cout << *iter_1 << "->";
                    }
                    cout << endl;
                    outfile << endl;

                    outfile << "agent_7 repair_path: ";
                    for (auto iter_2 = agent_best_pi_7.begin(); iter_2 != agent_best_pi_7.end(); ++iter_2)
                    {
                        outfile << *iter_2 << "->";
                        cout << *iter_2 << "->";
                    }
                    cout << endl;
                    outfile << endl;

                    outfile << "agent_8 repair_path: ";
                    for (auto iter_3 = agent_best_pi_8.begin(); iter_3 != agent_best_pi_8.end(); ++iter_3)
                    {
                        outfile << *iter_3 << "->";
                        cout << *iter_3 << "->";
                    }
                    cout << endl;
                    outfile << endl;

                    outfile << "agent_9 repair_path: ";
                    for (auto iter_2 = agent_best_pi_9.begin(); iter_2 != agent_best_pi_9.end(); ++iter_2)
                    {
                        outfile << *iter_2 << "->";
                        cout << *iter_2 << "->";
                    }
                    cout << endl;
                    outfile << endl;

                    outfile << "agent_10 repair_path: ";
                    for (auto iter_3 = agent_best_pi_10.begin(); iter_3 != agent_best_pi_10.end(); ++iter_3)
                    {
                        outfile << *iter_3 << "->";
                        cout << *iter_3 << "->";
                    }
                    cout << endl;
                    outfile << endl;
                    cout << connect_path << endl;
                    cout << "best_obj =" << best_obj << endl;
                    cout << "best_obj_RESCUE =" << best_obj_rescue << endl;
                    outfile.close();

                    char str2[20];
                    sprintf_s(str2, "g%d_c%d_DP", graph_index, AGENT_NUM);
                    string tempstring2(str2);
                    string filename2 = tempstring2 + "-object.txt";

                    ofstream outfile2;
                    outfile2.open(write_path + filename2, ofstream::app);
                    //outfile2 << "best_obj = " << best_obj << endl;
                    outfile2 << best_obj_rescue << endl;
                    outfile2.close();

                    char str3[20];
                    sprintf_s(str3, "g%d_c%d_DP", graph_index, AGENT_NUM);
                    string tempstring3(str3);
                    string filename3 = tempstring3 + "-time.txt";

                    ofstream outfile3;
                    outfile3.open(write_path + filename3, ofstream::app);
                    outfile3 << static_cast<double>((double)end_time - (double)start_time) / CLOCKS_PER_SEC * 1000 << endl;
                    outfile3.close();
                }
                //}
            }
        }
    }

    system("pause");
}