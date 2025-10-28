#include <stack>
#include <thread>
#include <mutex>
#include <windows.h>
#include <sstream>
#include <iomanip> 
#include<ctime>
#include "road.h"
#include "random.h"
#include<iostream>
//ACO
using namespace std;

mutex action_mutex; // Mutex for action set operations

mutex road_state_mutex; // Mutex for state space operations

mutex try_mutex;
double alpha = 0.003;
double beta = 0.02;
double rho = 0.2;
double QVAL = 1000;
int ant_num = 60;

map<int, bool> node_rescue;

map<double, Line> time_state;
map<double, Line> time_state_best;
map<double, double> time_state_1; // Delivery time for all demand points

/************************/
/* Enter basic graph information here */
/************************************************************************/
set<int> reserve_node_store; // Store reserve points

set<int> node_store; // Store road network nodes

set<int> const_demand_node_store; // Store original road network demand points before transformation
set<int> const_demand_node_store1; // Store original road network demand points before transformation, used for initialization

set<int> demand_node_store; // Damaged demand points
//set<int> demand_node_store1; // Demand points for material distribution

set<Line> edge_store; // Store road network edges

set<Line> const_damaged_edge_store; // Store original damaged edges before transformation, used for initialization
set<Line> damaged_edge_store;

map<int, double> max_allowed_dis_store_0; // Store maximum allowed distance
map<int, double> max_allowed_dis_store_1; // Store maximum allowed distance
map<int, double> max_allowed_dis_store_2; // Store maximum allowed distance
map<int, double> max_allowed_dis_store_3; // Store maximum allowed distance
map<int, double> max_allowed_dis_store_4; // Store maximum allowed distance

/***********************************************************************/
/*
The map container is a key-value pair collection. To have one label correspond to one state, the state set is used as the key.
Since the map requires unique keys, using the state set as the key ensures no duplicate states are inserted during storage.
This makes program processing easiest, rather than the conventional way (using label as the key).
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

set<Line> unexe_actions_set; // Record unexecuted action sets
set<int> unexe_actions_set1; // Record unexecuted action sets

map<string, double> pheromone_store_1; // map<"state_label-action", qvalue>
map<string, double> pheromone_store_2; // map<"state_label-action", qvalue>
map<string, double> pheromone_store_3; // map<"state_label-action", qvalue>
map<string, double> pheromone_store_4; // map<"state_label-action", qvalue>
map<string, double> pheromone_store_5; // map<"state_label-action", qvalue>

map<string, double> pheromone_store_6; // map<"state_label-action", qvalue>
map<string, double> pheromone_store_7; // map<"state_label-action", qvalue>
map<string, double> pheromone_store_8; // map<"state_label-action", qvalue>
map<string, double> pheromone_store_9; // map<"state_label-action", qvalue>
map<string, double> pheromone_store_10; // map<"state_label-action", qvalue>

map<Line, int> damaged_node_store; // map<damaged_edge, damaged_edge_id>

void initialize() {
	srand((unsigned)time(NULL)); // Initialize random function
}

void Initial_Graph()
{
	damaged_node_store.clear();

	pheromone_store_1.clear();
	pheromone_store_2.clear();
	pheromone_store_3.clear();
	pheromone_store_4.clear();
	pheromone_store_5.clear();

	pheromone_store_6.clear();
	pheromone_store_7.clear();
	pheromone_store_8.clear();
	pheromone_store_9.clear();
	pheromone_store_10.clear();

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

	demand_node_store.clear();

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

		if (importance != 0)	const_demand_node_store.insert(node);
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
		g.setImportance(index, 0); // Set reserve point importance
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

		g.setRescueTime(node, rescue_time); // Time spent delivering supplies at demand point

		//const_demand_node_store1.insert(node);

		node_rescue.insert(make_pair(node, false));
	}
	infile5.close();

	for (auto iter = const_damaged_edge_store.begin(); iter != const_damaged_edge_store.end(); ++iter)
	{
		g.setConnectionStatus(*iter, false);
	}
}

/* Enter road network change information here */

// Generate random integer in [a, b)
int getRandom(int lowerRound, int upperRound)
{
	int value;

	value = lowerRound + (int)(upperRound - lowerRound) * rand() / (RAND_MAX + 1);

	return value;
}

// Generate floating point number in (0, 1)
double getDoubleRandom()
{
	double double_rand;

	double_rand = rand() / double((double)RAND_MAX + 1);
	//double_rand = randomperca(); // Use normal distribution function to get (0,1) random value, more random than rand()
	//Original method: double_rand = rand() / double(RAND_MAX + 1);
	return double_rand;
}

// String split function, can split string s by given delimiter delim
// Use "-" as delimiter to separate "state_label" and "action" from "state_label-action"
void mySplitString(string s, string delim, vector<string>& ret)
{
	string::size_type split_pos = 0; // Record starting position of each split

	string::size_type index = s.find_first_of(delim, split_pos);
	/*
	s.find_first_of(arg1,arg2) finds the first occurrence of arg1 in s starting from index arg2, returns its position. Returns -1 if not found.
	*/

	while (index != -1)
	{
		ret.push_back(s.substr(split_pos, index - split_pos)); // Store characters between split position and delimiter position into ret
		//|state_label|-action, store state_label in ret
		//s.substr(arg1,arg2) string截断function: start from arg1,截取arg2 characters

		split_pos = index + 1; // Record starting position for next split
		index = s.find_first_of(delim, split_pos);
	}

	// If there are elements after the last delimiter
	if (s.length() - split_pos > 0)
	{
		ret.push_back(s.substr(split_pos));
	}
}

double get_value(int ant_index, int from, int to, double alpha, double beta, double dis) {
	string from_to = to_string(from) + "-" + to_string(to);
	double pheromone;

	switch (ant_index)
	{
	case 0:pheromone = pheromone_store_1[from_to]; break;
	case 1:pheromone = pheromone_store_2[from_to]; break;
	case 2:pheromone = pheromone_store_3[from_to]; break;
	case 3:pheromone = pheromone_store_4[from_to]; break;
	case 4:pheromone = pheromone_store_5[from_to]; break;
	case 5:pheromone = pheromone_store_6[from_to]; break;
	case 6:pheromone = pheromone_store_7[from_to]; break;
	case 7:pheromone = pheromone_store_8[from_to]; break;
	case 8:pheromone = pheromone_store_9[from_to]; break;
	case 9:pheromone = pheromone_store_10[from_to]; break;
	default:break;
	}
	return ((pow(pheromone, alpha)) * pow(1.0 / (dis + 1.0), beta));
}

void updatePheromone1(int ant_num, double rho, double QVAL, vector<double> total_cost_store, vector<vector<string>> ants_path_store_1, vector<vector<string>> ants_path_store_2,
	vector<vector<string>> ants_path_store_3, vector<vector<string>> ants_path_store_4, vector<vector<string>> ants_path_store_5) {
	//*Forgetting factor

	for (auto phero_iter = pheromone_store_6.begin(); phero_iter != pheromone_store_6.end(); ++phero_iter) {
		pheromone_store_6[phero_iter->first] *= (1.0 - rho);
	}

	for (auto phero_iter = pheromone_store_7.begin(); phero_iter != pheromone_store_7.end(); ++phero_iter) {
		pheromone_store_7[phero_iter->first] *= (1.0 - rho);
	}

	for (auto phero_iter = pheromone_store_8.begin(); phero_iter != pheromone_store_8.end(); ++phero_iter) {
		pheromone_store_8[phero_iter->first] *= (1.0 - rho);
	}

	for (auto phero_iter = pheromone_store_9.begin(); phero_iter != pheromone_store_9.end(); ++phero_iter) {
		pheromone_store_9[phero_iter->first] *= (1.0 - rho);
	}

	for (auto phero_iter = pheromone_store_10.begin(); phero_iter != pheromone_store_10.end(); ++phero_iter) {
		pheromone_store_10[phero_iter->first] *= (1.0 - rho);
	}

	for (int i = 0; i < ant_num; ++i) {
		double dis = total_cost_store[i]; // Objective function
		vector<string> from_to_store_1(ants_path_store_1[i]); // Agent1, first ant's action轨迹
		vector<string> from_to_store_2(ants_path_store_2[i]); // Agent2, second ant's action轨迹
		vector<string> from_to_store_3(ants_path_store_3[i]); // Agent3, third ant's action轨迹
		vector<string> from_to_store_4(ants_path_store_4[i]); // Agent4, second ant's action轨迹
		vector<string> from_to_store_5(ants_path_store_5[i]); // Agent5, third ant's action轨迹

		for (int j = 0; j < from_to_store_1.size(); ++j) {
			string from_to = from_to_store_1[j];
			pheromone_store_6[from_to] += (QVAL / dis);
		}
		for (int j = 0; j < from_to_store_2.size(); ++j) {
			string from_to = from_to_store_2[j];
			pheromone_store_7[from_to] += (QVAL / dis);
		}
		for (int j = 0; j < from_to_store_3.size(); ++j) {
			string from_to = from_to_store_3[j];
			pheromone_store_8[from_to] += (QVAL / dis);
		}
		for (int j = 0; j < from_to_store_4.size(); ++j) {
			string from_to = from_to_store_4[j];
			pheromone_store_9[from_to] += (QVAL / dis);
		}
		for (int j = 0; j < from_to_store_5.size(); ++j) {
			string from_to = from_to_store_5[j];
			pheromone_store_10[from_to] += (QVAL / dis);
		}
	}
}

void updatePheromone(int ant_num, double rho, double QVAL, vector<double> total_cost_store, vector<vector<string>> ants_path_store_1, vector<vector<string>> ants_path_store_2,
	vector<vector<string>> ants_path_store_3, vector<vector<string>> ants_path_store_4, vector<vector<string>> ants_path_store_5) {
	//*Forgetting factor

	for (auto phero_iter = pheromone_store_1.begin(); phero_iter != pheromone_store_1.end(); ++phero_iter) {
		pheromone_store_1[phero_iter->first] *= (1.0 - rho);
	}

	for (auto phero_iter = pheromone_store_2.begin(); phero_iter != pheromone_store_2.end(); ++phero_iter) {
		pheromone_store_2[phero_iter->first] *= (1.0 - rho);
	}

	for (auto phero_iter = pheromone_store_3.begin(); phero_iter != pheromone_store_3.end(); ++phero_iter) {
		pheromone_store_3[phero_iter->first] *= (1.0 - rho);
	}

	for (auto phero_iter = pheromone_store_4.begin(); phero_iter != pheromone_store_4.end(); ++phero_iter) {
		pheromone_store_4[phero_iter->first] *= (1.0 - rho);
	}

	for (auto phero_iter = pheromone_store_5.begin(); phero_iter != pheromone_store_5.end(); ++phero_iter) {
		pheromone_store_5[phero_iter->first] *= (1.0 - rho);
	}

	for (int i = 0; i < ant_num; ++i) {

		double dis = total_cost_store[i]; // Objective function
		//cout <<"ants_path_store_1[i] = " << ants_path_store_1[i].size() << endl;

		vector<string> from_to_store_1 = ants_path_store_1[i]; // Agent1, first ant's action轨迹
		vector<string> from_to_store_2 = ants_path_store_2[i]; // Agent2, second ant's action轨迹
		vector<string> from_to_store_3 = ants_path_store_3[i]; // Agent3, third ant's action轨迹
		vector<string> from_to_store_4 = ants_path_store_4[i]; // Agent2, second ant's action轨迹
		vector<string> from_to_store_5 = ants_path_store_5[i]; // Agent3, third ant's action轨迹

		for (int j = 0; j < from_to_store_1.size(); ++j) {
			string from_to = from_to_store_1[j];
			pheromone_store_1[from_to] += (QVAL / dis);
		}
		for (int j = 0; j < from_to_store_2.size(); ++j) {
			string from_to = from_to_store_2[j];
			pheromone_store_2[from_to] += (QVAL / dis);
		}
		for (int j = 0; j < from_to_store_3.size(); ++j) {
			string from_to = from_to_store_3[j];
			pheromone_store_3[from_to] += (QVAL / dis);
		}
		for (int j = 0; j < from_to_store_4.size(); ++j) {
			string from_to = from_to_store_4[j];
			pheromone_store_4[from_to] += (QVAL / dis);
		}
		for (int j = 0; j < from_to_store_5.size(); ++j) {
			string from_to = from_to_store_5[j];
			pheromone_store_5[from_to] += (QVAL / dis);
		}
	}
}

void Initial_State1(Graph g, int last_index, set<int>& actions_set)
{
	set<double> state_set;
	state_set.insert(last_index);
	actions_set.clear();

	vector<double> dis_to_0;

	g.shortestDis(last_index, dis_to_0); // Find shortest distance from all nodes to point 0, store in dis_to_0

	for (auto node_iter = unexe_actions_set1.begin(); node_iter != unexe_actions_set1.end(); ++node_iter)
	{
		//state_set.insert(*node_iter); // State table contains unsatisfied demand points	
		if (dis_to_0[*node_iter] != INF) // If demand point is reachable, store in action set 1
		{
			actions_set.insert(*node_iter);
			//cout << *node_iter << " ";
		}
	}

	//cout <<"*****************actions_set_2.size() = " << actions_set_1.size() <<" " << actions_set_2.size() <<" " << actions_set_3.size() << endl;
}

void Initial_State(Graph g, set<int>& demand_nodes_set, int last_index, set<Line>& actions_set)
{
	set<double> state_set;
	state_set.insert(last_index);
	actions_set.clear();
	vector<double> dis_to;

	g.shortestDis(last_index, dis_to); // Find shortest distance from all nodes to point 0, store in dis_to_0
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
	for (auto node_iter = demand_node_store.begin(); node_iter != demand_node_store.end(); ++node_iter) // Simplify action set
	{
		if (dis_to[*node_iter] < max_allowed_dis_store[*node_iter])
		{
			demand_nodes_set.erase(*node_iter);
		}
		else
		{
			//state_set.insert(*node_iter); // State table contains isolated demand points
		}
	}

	for (auto edge_iter = unexe_actions_set.begin(); edge_iter != unexe_actions_set.end(); ++edge_iter)
	{

		int action_value = (edge_iter->dest * 10000) + (edge_iter->src); // Action label
		//state_set.insert(action_value); // State table contains unprocessed damaged edges

		if (dis_to[edge_iter->src] != INF || dis_to[edge_iter->dest] != INF) // If demand point is reachable, store in action set 1
		{
			actions_set.insert(*edge_iter);
		}

	}
	//cout << "unexe_actions_set.size() = " << unexe_actions_set.size() << endl;
	//cout << "state_set.size() = " << state_set.size() << endl;
}

// from is current position, to possible reachable nodes
void getPossibleAction1(Graph g, int ant_index, int from, set<int> poss_action_set) {

	for (set<int>::iterator check_iter = poss_action_set.begin(); check_iter != poss_action_set.end(); ++check_iter) {

		int temp_to = *check_iter;
		string from_to = to_string(from) + "-" + to_string(temp_to);
		switch (ant_index)
		{
		case 0:pheromone_store_6.insert(make_pair(from_to, 1)); break;
		case 1:pheromone_store_7.insert(make_pair(from_to, 1)); break;
		case 2:pheromone_store_8.insert(make_pair(from_to, 1)); break;
		case 3:pheromone_store_9.insert(make_pair(from_to, 1)); break;
		case 4:pheromone_store_10.insert(make_pair(from_to, 1)); break;
		default:break;
		}
	}
}

void getPossibleAction(Graph g, int ant_index, int cur_index, int from, set<Line> poss_action_set, vector<double>& repair_travel_dis, map<Line, double>& poss_choose_action_dis, set<Line> V_ACT) {
	g.shortestDis(cur_index, repair_travel_dis);

	for (set<Line>::iterator check_iter = poss_action_set.begin(); check_iter != poss_action_set.end(); ++check_iter) {
		if (repair_travel_dis[check_iter->src] != INF || repair_travel_dis[check_iter->dest] != INF) {

			double dis = repair_travel_dis[check_iter->src] >= repair_travel_dis[check_iter->dest] ? repair_travel_dis[check_iter->dest] : repair_travel_dis[check_iter->src];
			if (V_ACT.find(*check_iter) == V_ACT.end()) poss_choose_action_dis.insert(make_pair(*check_iter, dis));

			int temp_to = damaged_node_store[*check_iter];
			string from_to = to_string(from) + "-" + to_string(temp_to);
			// Initialize pheromone table, if from_to doesn't exist, insert and initialize to 1;
			switch (ant_index)
			{
			case 0:pheromone_store_1.insert(make_pair(from_to, 1)); break;
			case 1:pheromone_store_2.insert(make_pair(from_to, 1)); break;
			case 2:pheromone_store_3.insert(make_pair(from_to, 1)); break;
			case 3:pheromone_store_4.insert(make_pair(from_to, 1)); break;
			case 4:pheromone_store_5.insert(make_pair(from_to, 1)); break;
			default:break;
			}
		}
	}
}

int ChooseAct1(int agent_index, int agent_num, int from, double alpha, double beta, map<int, double>& poss_choose_action_dis) {
	double total_value = 0;
	// Calculate total value
	for (auto iter = poss_choose_action_dis.begin(); iter != poss_choose_action_dis.end(); ++iter) {
		int to = iter->first;
		double dis = iter->second;
		total_value += get_value(agent_index + agent_num, from, to, alpha, beta, dis);
	}

	// Roulette wheel selection
	double fSlice = getDoubleRandom() * total_value;
	double fTotal = 0.0;
	int temp = 0;
	for (auto iter2 = poss_choose_action_dis.begin(); iter2 != poss_choose_action_dis.end(); ++iter2) {
		int to = iter2->first;
		double dis = iter2->second;
		fTotal += get_value(agent_index + agent_num, from, to, alpha, beta, dis);

		temp = iter2->first;

		if (fTotal > fSlice)	break;
	}

	return temp;
}

Line ChooseAct(int agent_index, int agent_num, int from, double alpha, double beta, map<Line, double>& poss_choose_action_dis) {
	double total_value = 0;
	// Calculate total value
	for (auto iter = poss_choose_action_dis.begin(); iter != poss_choose_action_dis.end(); ++iter) {
		int to = damaged_node_store[iter->first];
		double dis = iter->second;
		total_value += get_value(agent_index, from, to, alpha, beta, dis);
	}

	// Roulette wheel selection
	double fSlice = getDoubleRandom() * total_value;
	double fTotal = 0.0;
	int temp_src = 0, temp_dest = 0;
	for (auto iter2 = poss_choose_action_dis.begin(); iter2 != poss_choose_action_dis.end(); ++iter2) {
		int to = damaged_node_store[iter2->first];
		double dis = iter2->second;
		fTotal += get_value(agent_index, from, to, alpha, beta, dis);

		temp_src = iter2->first.src;
		temp_dest = iter2->first.dest;

		if (fTotal > fSlice)	break;
	}

	Line choose_action(temp_src, temp_dest);
	return choose_action;
}

void ExeAct1(int action_node, Graph& g, int agent_num, map<int, vector<double>>& dis_aft)
{
	for (int i = 0; i < agent_num; ++i)
	{
		vector<double> dis_array;
		g.shortestDis(i, dis_array);
		dis_aft[i] = dis_array;
	}
}

void ExeAct(Line action_line, Graph& g, int agent_num,
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
}

void EvalState(Graph g, int agent_num, set<int>& demand_nodes_set, double total_cost, map<int, vector<double>> dis_aft, double& weighted_repair_cost)
{
	// Filter reachable damaged edges from agent_unexe for next state + add agent's unexecuted segments to state

	set<int> demand_nodes_set_temp = demand_nodes_set;
	for (auto check_iter2 = demand_nodes_set_temp.begin(); check_iter2 != demand_nodes_set_temp.end(); ++check_iter2)
	{

		if (agent_num == 3 && dis_aft[0][*check_iter2] > max_allowed_dis_store_0[*check_iter2] && dis_aft[1][*check_iter2] > max_allowed_dis_store_1[*check_iter2] && dis_aft[2][*check_iter2] > max_allowed_dis_store_2[*check_iter2])
		{
			;
		}
		else if (agent_num == 5 && dis_aft[0][*check_iter2] > max_allowed_dis_store_0[*check_iter2] && dis_aft[1][*check_iter2] > max_allowed_dis_store_1[*check_iter2] && dis_aft[2][*check_iter2] > max_allowed_dis_store_2[*check_iter2]
			&& dis_aft[3][*check_iter2] > max_allowed_dis_store_3[*check_iter2] && dis_aft[4][*check_iter2] > max_allowed_dis_store_4[*check_iter2])
		{
			//state_set.insert(*check_iter2);
		}
		else {
			demand_nodes_set.erase(*check_iter2);
			weighted_repair_cost += g.getImportance(*check_iter2) * total_cost;
			//cout << *check_iter2 << " " << weighted_repair_cost << endl;
		}
	}
	//cout << "demand_nodes_set = " << demand_nodes_set.size() <<endl;
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

bool Policy1(Graph g, int agent_num, int& action_node, int agent_index, int& last_dest,
	double total_cost, set<int> actions_set, double& time, double& repair_cost, double& object_func_rescue, set<int> V_ACT_r)
{
	string state_action;
	actions_set.clear();

	map<int, double> poss_choose_action_dis; // map<poss_action_line, travel_from_to_dis>
	poss_choose_action_dis.clear();
	vector<double> dis_array;
	vector<Line> s;

	for (auto iter = time_state.begin(); iter != time_state.end(); iter++) {
		if (total_cost >= iter->first) {
			g.setConnectionStatus(iter->second, true);
			s.push_back(iter->second);
		}
		else break;
	}
	//cout << "iter->second = " << s.size() << endl;
	g.shortestDis(last_dest, dis_array);

	for (auto iter = s.begin(); iter != s.end(); iter++) g.setConnectionStatus(*iter, false);

	for (auto check_iter2 = unexe_actions_set1.begin(); check_iter2 != unexe_actions_set1.end(); ++check_iter2)
	{
		if (dis_array[*check_iter2] != INF && V_ACT_r.find(*check_iter2) == V_ACT_r.end())
		{
			actions_set.insert(*check_iter2); // Reachable demand points			
			double dis = dis_array[*check_iter2];
			poss_choose_action_dis.insert(make_pair(*check_iter2, dis));

			// Filter reachable and unrepaired edges, get all corresponding Q(s,a) values in current state s			
		}
	}
	getPossibleAction1(g, agent_index, last_dest, actions_set);

	if (poss_choose_action_dis.size() != 0) {

		// Greedy decision to select action
		action_node = ChooseAct1(agent_index, agent_num, last_dest, alpha, beta, poss_choose_action_dis);
	}

	// Determine maximum Q value in state s and corresponding <state-action pair>
	else {
		time = total_cost + 5;
		//cout << "Time extended!" << endl;
		return false;
	}
	//cout <<"agent_index = "<< agent_index<< "Select action:" <<  action_node << endl;
	// Determine time cost required to execute selected action
	repair_cost = dis_array[action_node];

	double tmp_cost = total_cost + repair_cost;
	unexe_actions_set1.erase(action_node);

	if (!node_rescue[action_node]) tmp_cost += g.getRescueTime(action_node); // If demand point not satisfied, add delivery time

	//cout << "agent_index = " << agent_index << "total_cost = " << total_cost << "repair_cost = "<< repair_cost << endl;
	//last_dest = action_node; // Update current position
	time = tmp_cost;
	object_func_rescue = object_func_rescue + time * g.getImportance(action_node);
	//cout << time <<" "<< g.getImportance(action_node)<<" " << object_func_rescue << endl;
	return true;
}

// agent_index action selection at time total_cost, actions_set is public action set unexe_actions_set
bool Policy(Graph g, int agent_num, Line& action_line, int agent_index, int& last_dest, int from,
	double total_cost, set<Line> actions_set, double& time, set<Line> V_ACT)
{
	int next_dest;
	actions_set.clear();
	actions_set = unexe_actions_set;
	vector<double> repair_travel_dis;
	vector<double> dis_array;
	map<Line, double> poss_choose_action_dis; // map<poss_action_line, travel_from_to_dis>
	repair_travel_dis.clear();
	poss_choose_action_dis.clear();
	dis_array.clear();
	//cout << "actions_set = " << actions_set.size() << endl;
	// Filter unselected damaged edges

	getPossibleAction(g, agent_index, last_dest, from, unexe_actions_set, repair_travel_dis, poss_choose_action_dis, V_ACT);

	if (poss_choose_action_dis.size() == 0) {
		//cout << "No path available! time = " << time << endl;
		time = total_cost + 5;
		return false;
	}

	// Determine maximum Q value in state s and corresponding <state-action pair>

	// Greedy decision to select action
	action_line = ChooseAct(agent_index, agent_num, last_dest, alpha, beta, poss_choose_action_dis);
	// Determine time cost required to execute selected action
	unexe_actions_set.erase(action_line);

	g.shortestDis(last_dest, dis_array);

	next_dest = dis_array[action_line.dest] > dis_array[action_line.src] ? action_line.dest : action_line.src;

	double repair_cost = dis_array[action_line.dest] > dis_array[action_line.src] ? dis_array[action_line.src] : dis_array[action_line.dest];

	repair_cost += g.getRepairTime(action_line) + g.getWeight(action_line);

	time = total_cost + repair_cost;
	//cout << "time = " << time << "repair_cost = " << repair_cost << endl;
	last_dest = next_dest; // Update position

	return true;
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

// Repair teams work independently
double Pthread_learning(Graph g, int agent_num, double alpha, double gamma, double lambda,
	vector<Line>& pi_1, vector<Line>& pi_2, vector<Line>& pi_3, vector<Line>& pi_4, vector<Line>& pi_5, vector<int>& pi_6, vector<int>& pi_7, vector<int>& pi_8, vector<int>& pi_9, vector<int>& pi_10,
	vector<string>& from_to_store_1, vector<string>& from_to_store_2, vector<string>& from_to_store_3, vector<string>& from_to_store_4, vector<string>& from_to_store_5, vector<string>& from_to_store_6,
	vector<string>& from_to_store_7, vector<string>& from_to_store_8, vector<string>& from_to_store_9, vector<string>& from_to_store_10,
	set<int> demand_nodes_set, set<Line> actions_unexe, double& object_func, double& object_func_rescue, map<double, double>& time_state_1)
{
	set<Line> actions_set_1, actions_set_2, actions_set_3, actions_set_4, actions_set_5;
	set<int> actions_set_6, actions_set_7, actions_set_8, actions_set_9, actions_set_10;
	double weighted_repair_cost = 0;
	double weighted_dis_cost = 0;
	time_state.clear();
	time_state_1.clear();

	double time_1 = 0, time_2 = 0, time_3 = 0, time = 0, time_4 = 0, time_5 = 0, time_6 = 0, time_7 = 0, time_8 = 0, time_9 = 0, time_10 = 0;

	bool exe_flag_1 = false, exe_flag_2 = false, exe_flag_3 = false, exe_flag_4 = false, exe_flag_5 = false; // Flag indicating whether previous action was executed

	bool exe_flag_6 = false, exe_flag_7 = false, exe_flag_8 = false, exe_flag_9 = false, exe_flag_10 = false; // Flag indicating whether previous action was executed

	double total_cost_1 = 0, total_cost_2 = 0, total_cost_3 = 0, total_cost_4 = 0, total_cost_5 = 0;

	double total_cost_6 = 0, total_cost_7 = 0, total_cost_8 = 0, total_cost_9 = 0, total_cost_10 = 0;

	bool act_flag_1 = false, act_flag_2 = false, act_flag_3 = false, act_flag_4 = false, act_flag_5 = false;

	bool act_flag_6 = false, act_flag_7 = false, act_flag_8 = false, act_flag_9 = false, act_flag_10 = false; // Flag indicating whether new action needs to be pushed to stack

	int last_dest_1 = 0, last_dest_2 = 1, last_dest_3 = 2, last_dest_4 = 3, last_dest_5 = 4, last_dest_6 = 0, last_dest_7 = 1, last_dest_8 = 2, last_dest_9 = 3, last_dest_10 = 4;

	Line act_1(0, 0), act_2(1, 1), act_3(2, 2), act_4(3, 3), act_5(4, 4);

	int act_6 = 0, act_7 = 1, act_8 = 2, act_9 = 3, act_10 = 4;

	double repair_cost_1 = 0, repair_cost_2 = 0, repair_cost_3 = 0, repair_cost_4 = 0, repair_cost_5 = 0;

	int from1 = 0, from2 = 0, from3 = 0, from4 = 0, from5 = 0, from6 = 0, from7 = 0, from8 = 0, from9 = 0, from10 = 0; // Current position is last_dest, repaired damaged segment number is from, next repair number is to
	//stack<Line> S_ACT; // Monotonic stack
	for (int i = 0; i < agent_num; i++) {
		switch (i)
		{
		case 0: {
			Initial_State(g, demand_nodes_set, 0, actions_set_1);
			exe_flag_1 = true;
			exe_flag_6 = true;
			break;
		}
		case 1: {
			Initial_State(g, demand_nodes_set, 1, actions_set_2);
			exe_flag_2 = true;
			exe_flag_7 = true;
			break;
		}
		case 2: {
			Initial_State(g, demand_nodes_set, 2, actions_set_3);
			exe_flag_3 = true;
			exe_flag_8 = true;
			break;
		}
		case 3: {
			Initial_State(g, demand_nodes_set, 3, actions_set_4);
			exe_flag_4 = true;
			exe_flag_9 = true;
			break;
		}
		case 4: {
			Initial_State(g, demand_nodes_set, 4, actions_set_5);
			exe_flag_5 = true;
			exe_flag_10 = true;
			break;
		}
		default:
			break;
		}
	}
	vector<Line> V_ACT; // Buffer storing actions to be executed
	map<Line, int> action_agent; // Record action corresponding repair team
	map<Line, double> action_fine; // Record corresponding action execution time
	map<int, vector<double>> dis_pre, dis_aft;

	vector<int> V_ACT_r;
	map<int, int> action_agent_r; // Record action corresponding repair team
	map<int, double> action_fine_r; // Record corresponding action execution time
	bool quit_flag = false;

	set<int> demand_nodes_set_rescue = unexe_actions_set1;
	set<Line> v;
	while (!demand_nodes_set.empty())
	{
		dis_pre.clear();
		dis_aft.clear();
		Line exe_action(0, 0); // Record action to be executed in buffer

		vector<double> dis_array;
		act_flag_1 = act_flag_2 = act_flag_3 = act_flag_4 = act_flag_5 = false;
		bool flag = false;

		if (exe_flag_1)
		{
			flag = Policy(g, agent_num, act_1, 0, last_dest_1, from1,
				total_cost_1, actions_set_1, time_1, v);
			v.insert(act_1);
			action_agent[act_1] = 1; // Set repair team executing action act_1

			action_fine[act_1] = time_1; // Store time after executing act_1
			exe_flag_1 = false;
			act_flag_1 = true;
		}
		if (exe_flag_2)
		{
			flag = Policy(g, agent_num, act_2, 1, last_dest_2, from2,
				total_cost_2, actions_set_2, time_2, v);
			v.insert(act_2);
			action_agent[act_2] = 2;

			action_fine[act_2] = time_2;
			exe_flag_2 = false;
			act_flag_2 = true;
		}
		if (exe_flag_3)
		{
			flag = Policy(g, agent_num, act_3, 2, last_dest_3, from3,
				total_cost_3, actions_set_3, time_3, v);
			v.insert(act_3);
			action_agent[act_3] = 3;

			action_fine[act_3] = time_3;
			exe_flag_3 = false;
			act_flag_3 = true;
		}
		if (exe_flag_4)
		{
			flag = Policy(g, agent_num, act_4, 3, last_dest_4, from4,
				total_cost_4, actions_set_4, time_4, v);
			v.insert(act_4);
			action_agent[act_4] = 4;

			action_fine[act_4] = time_4;
			exe_flag_4 = false;
			act_flag_4 = true;
		}
		if (exe_flag_5)
		{
			flag = Policy(g, agent_num, act_5, 4, last_dest_5, from5,
				total_cost_5, actions_set_5, time_5, v);
			v.insert(act_5);
			action_agent[act_5] = 5;

			action_fine[act_5] = time_5;
			exe_flag_5 = false;
			act_flag_5 = true;
		}

		// Select action to execute this time
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

		/************************************************************************/

		if (V_ACT.size() > 1)
		{
			Bubble(V_ACT, action_fine);
			int min_index = V_ACT.size(); // Index with minimum time consumption
			exe_action = V_ACT[min_index - 1]; // Currently least time-consuming action, take one action each time
			auto it = V_ACT.begin() + min_index - 1;
			V_ACT.erase(it);
		}
		else if (V_ACT.size() == 1)
		{
			exe_action = V_ACT[0];
			V_ACT.clear();
		}

		time_state.insert(make_pair(action_fine[exe_action], exe_action));
		//cout << "exe_action = " << exe_action.src<<"-"<< exe_action.dest << endl;
		//cout << "action_fine[exe_action] = " << action_fine[exe_action] << endl;
		// Set flag corresponding to executed action to 1
		//cout << "action_agent[exe_action] = " << action_agent[exe_action] << endl;
		switch (action_agent[exe_action])
		{
		case 1: {
			double last_total_cost = total_cost_1;
			total_cost_1 = time_1;
			exe_flag_1 = true;
			ExeAct(exe_action, g, agent_num, dis_pre, dis_aft); // Execute action		
			EvalState(g, agent_num, demand_nodes_set, total_cost_1, dis_aft, weighted_repair_cost);
			if (flag) {
				pi_1.push_back(exe_action); // Record agent's action轨迹
				int to = damaged_node_store[exe_action];
				string from_to1 = to_string(from1) + "-" + to_string(to);
				from_to_store_1.push_back(from_to1);
				pheromone_store_1.insert(make_pair(from_to1, 1));
				from1 = to;
			}
			break;
		}
		case 2: {
			double last_total_cost = total_cost_2;
			total_cost_2 = time_2;
			exe_flag_2 = true;
			ExeAct(exe_action, g, agent_num, dis_pre, dis_aft); // Execute action
			EvalState(g, agent_num, demand_nodes_set, total_cost_2, dis_aft, weighted_repair_cost);

			if (flag) {
				pi_2.push_back(exe_action);
				int to = damaged_node_store[exe_action];
				string from_to2 = to_string(from2) + "-" + to_string(to);
				from_to_store_2.push_back(from_to2);
				pheromone_store_2.insert(make_pair(from_to2, 1));
				from2 = to;
			}
			break;
		}
		case 3: {
			double last_total_cost = total_cost_3;
			total_cost_3 = time_3;
			exe_flag_3 = true;
			ExeAct(exe_action, g, agent_num, dis_pre, dis_aft); // Execute action

			EvalState(g, agent_num, demand_nodes_set, total_cost_3, dis_aft, weighted_repair_cost);

			if (flag) {
				pi_3.push_back(exe_action);
				int to = damaged_node_store[exe_action];
				string from_to3 = to_string(from3) + "-" + to_string(to);
				from_to_store_3.push_back(from_to3);
				pheromone_store_3.insert(make_pair(from_to3, 1));
				from3 = to;
			}
			break;
		}
		case 4: {
			double last_total_cost = total_cost_4;
			total_cost_4 = time_4;
			exe_flag_4 = true;
			ExeAct(exe_action, g, agent_num, dis_pre, dis_aft); // Execute action
			EvalState(g, agent_num, demand_nodes_set, total_cost_4, dis_aft, weighted_repair_cost);

			if (flag) {
				pi_4.push_back(exe_action);
				int to = damaged_node_store[exe_action];
				string from_to4 = to_string(from4) + "-" + to_string(to);
				from_to_store_4.push_back(from_to4);
				pheromone_store_4.insert(make_pair(from_to4, 1));
				from4 = to;
			}
			break;
		}
		case 5: {
			double last_total_cost = total_cost_5;
			total_cost_5 = time_5;
			exe_flag_5 = true;
			ExeAct(exe_action, g, agent_num, dis_pre, dis_aft); // Execute action

			EvalState(g, agent_num, demand_nodes_set, total_cost_5, dis_aft, weighted_repair_cost);

			if (flag) {
				pi_5.push_back(exe_action);
				int to = damaged_node_store[exe_action];
				string from_to5 = to_string(from5) + "-" + to_string(to);
				from_to_store_5.push_back(from_to5);
				pheromone_store_5.insert(make_pair(from_to5, 1));
				from5 = to;
			}
			break;
		}
		default:break;
		}

		// Calculate corresponding time-weighted objective function
		if (demand_nodes_set.size() == 4 || demand_nodes_set.size() == 8) initialize();
		if (demand_nodes_set.empty()) {
			//cout << "Repair completed!" << endl;
			exe_flag_1 = exe_flag_2 = exe_flag_3 = exe_flag_4 = exe_flag_5 = false;
		}
	}
	if (!test(g)) {
		object_func_rescue = INF;
		cout << "Repair strategy error!" << endl;
	}
	else {
		for (set<Line>::iterator damaged_iter = damaged_edge_store.begin(); damaged_iter != damaged_edge_store.end(); ++damaged_iter)
			g.setConnectionStatus(*damaged_iter, false);
		int i = 1;
		set<int> v_r;
		while (!demand_nodes_set_rescue.empty()) {

			act_flag_6 = act_flag_7 = act_flag_8 = act_flag_9 = act_flag_10 = false;
			bool quit_flag = false;
			int exe_act = 0; // Record action to be executed in buffer

			if (exe_flag_6)
			{
				quit_flag = Policy1(g, agent_num, act_6, 0, last_dest_6,
					total_cost_6, actions_set_6, time_6, repair_cost_1, object_func_rescue, v_r); // Select execution action from action_sets
				v_r.insert(act_6);
				action_agent_r[act_6] = 1; // Set repair team executing action act_1
				action_fine_r[act_6] = time_6; // Store time after executing act_1
				exe_flag_6 = false;
				act_flag_6 = true;
			}
			if (exe_flag_7)
			{
				quit_flag = Policy1(g, agent_num, act_7, 1, last_dest_7,
					total_cost_7, actions_set_7, time_7, repair_cost_2, object_func_rescue, v_r); // Select execution action from action_sets
				v_r.insert(act_7);
				action_agent_r[act_7] = 2; // Set repair team executing action act_1=1
				action_fine_r[act_7] = time_7; // Store time after executing act_1
				exe_flag_7 = false;
				act_flag_7 = true;
			}
			if (exe_flag_8)
			{
				quit_flag = Policy1(g, agent_num, act_8, 2, last_dest_8,
					total_cost_8, actions_set_8, time_8, repair_cost_3, object_func_rescue, v_r); // Select execution action from action_sets
				v_r.insert(act_8);
				action_agent_r[act_8] = 3; // Set repair team executing action act_1=1
				action_fine_r[act_8] = time_8; // Store time after executing act_1			
				exe_flag_8 = false;
				act_flag_8 = true;
			}
			if (exe_flag_9)
			{
				quit_flag = Policy1(g, agent_num, act_9, 3, last_dest_9,
					total_cost_9, actions_set_9, time_9, repair_cost_4, object_func_rescue, v_r); // Select execution action from action_sets
				v_r.insert(act_9);
				action_agent_r[act_9] = 4; // Set repair team executing action act_1=1
				action_fine_r[act_9] = time_9; // Store time after executing act_1
				exe_flag_9 = false;
				act_flag_9 = true;
			}
			if (exe_flag_10)
			{
				quit_flag = Policy1(g, agent_num, act_10, 4, last_dest_10,
					total_cost_10, actions_set_10, time_10, repair_cost_5, object_func_rescue, v_r); // Select execution action from action_sets
				v_r.insert(act_10);
				action_agent_r[act_10] = 5; // Set repair team executing action act_1=1
				action_fine_r[act_10] = time_10; // Store time after executing act_1			
				exe_flag_10 = false;
				act_flag_10 = true;
			}

			// Select action to execute this time
			if (act_flag_6)
				V_ACT_r.push_back(act_6); // V_ACT.size()=333.......321

			if (act_flag_7)
				V_ACT_r.push_back(act_7);

			if (act_flag_8)
				V_ACT_r.push_back(act_8);

			if (act_flag_9)
				V_ACT_r.push_back(act_9);

			if (act_flag_10)
				V_ACT_r.push_back(act_10);

			if (V_ACT_r.size() > 1)
			{
				Bubble1(V_ACT_r, action_fine_r);
				int min_index = V_ACT_r.size(); // Index with minimum time consumption
				exe_act = V_ACT_r[min_index - 1]; // Currently least time-consuming action, take one action each time
				auto it = V_ACT_r.begin() + min_index - 1;
				V_ACT_r.erase(it);
			}
			else if (V_ACT_r.size() == 1)
			{
				exe_act = V_ACT_r[0];
				V_ACT_r.clear();
			}
			//cout <<"exe_action = " << exe_act << "  act_4 = " << act_4 << "  act_5 = " << act_5 << "  act_6 = " << act_6 << endl;

			if (node_rescue[exe_act] == false) time_state_1.insert(make_pair(action_fine_r[exe_act], g.getImportance(exe_act)));
			node_rescue[exe_act] = true; // Demand point satisfied = true
			demand_nodes_set_rescue.erase(exe_act);
			//cout << "unexe_actions_set1 = " << unexe_actions_set1.size() << " demand_nodes_set_rescue  = " << demand_nodes_set_rescue.size() << endl;

			switch (action_agent_r[exe_act])
			{
			case 1: {
				total_cost_6 = time_6;
				exe_flag_6 = true;
				//cout << "Transport team 1" << endl;	
				pi_6.push_back(exe_act); // Store this action
				if (quit_flag) {
					//object_func_rescue += action_fine_r[exe_act] * g.getImportance(exe_act);
					double dis = repair_cost_1;
					string from_to6 = to_string(last_dest_6) + "-" + to_string(act_6);
					from_to_store_6.push_back(from_to6);
					pheromone_store_6.insert(make_pair(from_to6, 1));
					last_dest_6 = act_6;
				}
				break;
			}
			case 2: {
				total_cost_7 = time_7;
				exe_flag_7 = true;
				//cout << "Transport team 2" << endl;	
				pi_7.push_back(exe_act); // Store this action
				if (quit_flag) {
					//object_func_rescue += action_fine_r[exe_act] * g.getImportance(exe_act);
					double dis = repair_cost_2;
					string from_to7 = to_string(last_dest_7) + "-" + to_string(act_7);
					from_to_store_7.push_back(from_to7);
					pheromone_store_7.insert(make_pair(from_to7, 1));
					last_dest_7 = act_7;
				}
				break;

			}
			case 3: {
				total_cost_8 = time_8;
				exe_flag_8 = true;
				//cout << "Transport team 3" << endl;	
				pi_8.push_back(exe_act); // Store this action
				if (quit_flag) {
					// += action_fine_r[exe_act] * g.getImportance(exe_act);
					double dis = repair_cost_3;
					string from_to8 = to_string(last_dest_8) + "-" + to_string(act_8);
					from_to_store_8.push_back(from_to8);
					pheromone_store_4.insert(make_pair(from_to8, 1));
					last_dest_8 = act_8;
				}
				break;
			}
			case 4: {
				total_cost_9 = time_9;
				exe_flag_9 = true;
				//cout << "Transport team 4" << endl;	
				pi_9.push_back(exe_act); // Store this action
				if (quit_flag) {
					//object_func_rescue += action_fine_r[exe_act] * g.getImportance(exe_act);
					double dis = repair_cost_4;
					string from_to9 = to_string(last_dest_9) + "-" + to_string(act_9);
					from_to_store_9.push_back(from_to9);
					pheromone_store_9.insert(make_pair(from_to9, 1));
					last_dest_9 = act_9;
				}
				break;
			}
			case 5: {
				total_cost_10 = time_10;
				exe_flag_10 = true;
				//cout << "Transport team 5" << endl;	
				pi_10.push_back(exe_act); // Store this action
				if (quit_flag) {
					//object_func_rescue += action_fine_r[exe_act] * g.getImportance(exe_act);
					double dis = repair_cost_5;
					string from_to10 = to_string(last_dest_10) + "-" + to_string(act_10);
					from_to_store_10.push_back(from_to10);
					pheromone_store_10.insert(make_pair(from_to10, 1));
					last_dest_10 = act_10;
				}
				break;
			}
			default:break;
			}
			if (demand_nodes_set_rescue.empty()) {
				//cout << "Transport completed!" << endl;
				exe_flag_6 = exe_flag_7 = exe_flag_8 = exe_flag_9 = exe_flag_10 = false;
			}
		}
	}

	vector<double> dis_to_0;
	vector<double> dis_to_1;
	vector<double> dis_to_2;
	vector<double> dis_to_3;
	vector<double> dis_to_4;
	dis_to_0.clear();
	dis_to_1.clear();
	dis_to_2.clear();
	dis_to_3.clear();
	dis_to_4.clear();

	g.shortestDis(0, dis_to_0); // Find shortest distance from all nodes to point 0, store in dis_to_0
	g.shortestDis(1, dis_to_1); // Find shortest distance from all nodes to point 1, store in dis_to_1
	g.shortestDis(2, dis_to_2); // Find shortest distance from all nodes to point 2, store in dis_to_2
	g.shortestDis(3, dis_to_3); // Find shortest distance from all nodes to point 3, store in dis_to_3
	g.shortestDis(4, dis_to_4); // Find shortest distance from all nodes to point 4, store in dis_to_4

	for (auto node_iter = unexe_actions_set1.begin(); node_iter != unexe_actions_set1.end(); ++node_iter)
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

	//cout << "object_func_rescue = " << object_func_rescue << endl;
	//cout << "object_func = " << object_func << endl;
	return object_func_rescue;
}

// Specify training cycles, episode: cycle
void episode_multi_Q(Graph g, int episode, int agent_num, int f_clock, double epsilon, double alpha, double gamma, double lambda, double& best_obj, double& best_obj_rescue, int& best_i)
{
	int temp_best_i;
	initialize();

	for (auto iter = damaged_edge_store.begin(); iter != damaged_edge_store.end(); ++iter) {
		int damaged_id = damaged_node_store.size();
		damaged_node_store.insert(make_pair(*iter, damaged_id + 1));
	}
	for (int i = 0; i < episode; ++i)
	{
		cout << i + 1 << " = ";
		int ant_index = 0;
		double temp_best_obj = INF, temp_best_obj_rescue = INF;
		vector<double> total_cost_store, total_cost_rescue_s;
		vector<vector<string>> ants_path_store_1, ants_path_store_2, ants_path_store_3, ants_path_store_4, ants_path_store_5, ants_path_store_6, ants_path_store_7, ants_path_store_8, ants_path_store_9, ants_path_store_10;
		ants_path_store_1.clear();
		ants_path_store_2.clear();
		ants_path_store_3.clear();
		ants_path_store_4.clear();
		ants_path_store_5.clear();
		ants_path_store_6.clear();
		ants_path_store_7.clear();
		ants_path_store_8.clear();
		ants_path_store_9.clear();
		ants_path_store_10.clear();
		map<double, double> time_state_2;
		map<double, Line> time_state_0;
		while (ant_index < ant_num) {

			//cout << " Number of ants = " << ant_index + 1 << endl;
			// Initialize damaged paths to false
			for (set<Line>::iterator damaged_iter = damaged_edge_store.begin(); damaged_iter != damaged_edge_store.end(); ++damaged_iter)
				g.setConnectionStatus(*damaged_iter, false);

			for (set<int>::iterator damaged_iter = demand_node_store.begin(); damaged_iter != demand_node_store.end(); ++damaged_iter)
				node_rescue[*damaged_iter] = false;

			set<int> demand_nodes_set;
			demand_nodes_set.clear();
			demand_nodes_set = demand_node_store;

			unexe_actions_set.clear();
			unexe_actions_set1.clear();
			unexe_actions_set = damaged_edge_store; // Store damaged edges
			unexe_actions_set1 = const_demand_node_store;

			vector<string> path_1, path_2, path_3, path_4, path_5, path_6, path_7, path_8, path_9, path_10;
			vector<Line> T_pi_1, T_pi_2, T_pi_3, T_pi_4, T_pi_5;
			vector<int> T_pi_6, T_pi_7, T_pi_8, T_pi_9, T_pi_10;

			double object_func = 0, object_func_rescue = 0;
			map<double, double> time_state_r;
			Pthread_learning(g, agent_num, alpha, gamma, lambda,
				T_pi_1, T_pi_2, T_pi_3, T_pi_4, T_pi_5, T_pi_6, T_pi_7, T_pi_8, T_pi_9, T_pi_10, path_1, path_2, path_3, path_4, path_5, path_6, path_7, path_8, path_9, path_10,
				demand_nodes_set, unexe_actions_set, object_func, object_func_rescue, time_state_r);

			ants_path_store_1.push_back(path_1);
			ants_path_store_2.push_back(path_2);
			ants_path_store_3.push_back(path_3);
			ants_path_store_4.push_back(path_4);
			ants_path_store_5.push_back(path_5);
			ants_path_store_6.push_back(path_6);
			ants_path_store_7.push_back(path_7);
			ants_path_store_8.push_back(path_8);
			ants_path_store_9.push_back(path_9);
			ants_path_store_10.push_back(path_10);

			total_cost_store.push_back(object_func); // Store objective function value of each ant group, used to update pheromone after each iteration
			total_cost_rescue_s.push_back(object_func_rescue);

			if (object_func < temp_best_obj)
			{
				agent_best_pi_1.clear();
				agent_best_pi_2.clear();
				agent_best_pi_3.clear();
				agent_best_pi_4.clear();
				agent_best_pi_5.clear();
				agent_best_pi_1 = T_pi_1;
				agent_best_pi_2 = T_pi_2;
				agent_best_pi_3 = T_pi_3;
				agent_best_pi_4 = T_pi_4;
				agent_best_pi_5 = T_pi_5;

				temp_best_obj = object_func;
			}

			if (object_func_rescue < temp_best_obj_rescue)
			{
				time_state_0 = time_state;
				time_state_2 = time_state_r;
				temp_best_i = ant_index;
				agent_best_pi_6.clear();
				agent_best_pi_7.clear();
				agent_best_pi_8.clear();
				agent_best_pi_9.clear();
				agent_best_pi_10.clear();
				agent_best_pi_6 = T_pi_6;
				agent_best_pi_7 = T_pi_7;
				agent_best_pi_8 = T_pi_8;
				agent_best_pi_9 = T_pi_9;
				agent_best_pi_10 = T_pi_10;

				temp_best_obj_rescue = object_func_rescue;
			}

			++ant_index;

		}
		cout << endl;
		cout << "Best ant this round temp_best_i = " << temp_best_i + 1 << "  temp_best_obj_rescue = " << temp_best_obj_rescue << "  temp_best_obj = " << temp_best_obj << endl;
		updatePheromone(ant_num, rho, QVAL, total_cost_store, ants_path_store_1, ants_path_store_2, ants_path_store_3, ants_path_store_4, ants_path_store_5);
		updatePheromone1(ant_num, rho, QVAL, total_cost_rescue_s, ants_path_store_6, ants_path_store_7, ants_path_store_8, ants_path_store_9, ants_path_store_10);

		if (temp_best_obj < best_obj) best_obj = temp_best_obj;

		if (temp_best_obj_rescue < best_obj_rescue)
		{
			time_state_best = time_state_0;
			time_state_1 = time_state_2;
			best_obj_rescue = temp_best_obj_rescue;
			best_i = i;
		}
	}
	cout << " Repair schedule: " << time_state_best.size() << endl;
	for (auto iter = time_state_best.begin(); iter != time_state_best.end(); ++iter)
	{
		cout << iter->first << " ";
	}
	cout << endl;
	cout << " Transport schedule: " << time_state_1.size() << endl;
	for (auto iter = time_state_1.begin(); iter != time_state_1.end(); ++iter)
	{
		cout << iter->first << " ";
	}
	cout << endl;
	cout << "Best ant best_i = " << best_i + 1 << "  best_obj_rescue = " << best_obj_rescue << "  best_obj = " << best_obj << endl;
}

int main()
{
	vector<int> node_num_store = { 80 };
	vector<int> edge_num_store = { 120 };
	vector<double> demand_pro_store = { 0.4,0.5,0.6 };
	vector<double> damaged_pro_store = { 0.5,0.6,0.7 };

	vector<bool> changed_demand_node_flag_store = { false };
	vector<bool> changed_damaged_edge_flag_store = { true };

	map<int, int> episode_store =
	{
		{ 40, 200 },
		{ 60, 200 },
		{ 80, 200 },
	};
	map<int, int> AGENT_NUM_store =
	{
		{ 40, 3 },
		{ 60, 5 },
		{ 80, 5 },
	};

	int GRAPH_NUM = 1;

	int CHANGE_GRAPH = 10;

	int TRAIN_TIME = 1;

	int f_clock = 150;

	double v_raito = 0.1;

	double epsilon = 0.9, alpha = 0.4, gamma = 0.2, lambda = 0.1;

	string common_path = "E:\\Test_EXP\\90 Road Network Cases\\";//the path of the road network dataset

	string common_path2 = "E:\\Test_EXP\\ACO\\";//Default storage path for experimental results：
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

				//for (int alter_index = 1; alter_index <= CHANGE_GRAPH; ++alter_index)
				//{
				Graph g(node_num, v_raito);

				createGraph(g, connect_path, AGENT_NUM);

				for (int train_index = 0; train_index < TRAIN_TIME; ++train_index)
				{

					Initial_Graph();

					cout << connect_path << endl;

					double best_obj = INF, best_obj_rescue = INF;
					int episode = episode_store[node_num], best_i = 0;

					clock_t start_time = clock();

					episode_multi_Q(g, episode, AGENT_NUM, f_clock, epsilon, alpha, gamma, lambda, best_obj, best_obj_rescue, best_i);

					clock_t end_time = clock();

					char str[20];
					sprintf_s(str, "g%d_c%d_ACO", graph_index, AGENT_NUM);
					string tempstring(str);
					string filename = tempstring + ".txt";

					string write_path = common_path2 + node_edge + demand_damage + to_string(AGENT_NUM) + "_agent\\" + "Data\\";

					ofstream outfile;
					outfile.open(write_path + filename, ofstream::app);
					outfile << "ACO:" << endl;
					outfile << "node_num = " << node_num << "edge_num = " << edge_num << ";" << demand_node_pro << "_" << damaged_edge_pro << endl;
					outfile << "/***********************ACO result***********************/" << endl;
					outfile << "solution:" << endl;
					outfile << " time_state.size() = " << time_state_best.size() << endl;
					for (auto iter = time_state_best.begin(); iter != time_state_best.end(); ++iter)
					{
						outfile << iter->first << " ,";
					}
					outfile << endl;
					outfile << "time_state_1.size() = " << time_state_1.size() << endl;
					for (auto iter = time_state_1.begin(); iter != time_state_1.end(); ++iter)
					{
						outfile << iter->first << " ,";
					}
					outfile << endl;

					outfile << "agent_1 repair_path:";
					for (auto iter_1 = agent_best_pi_1.begin(); iter_1 != agent_best_pi_1.end(); ++iter_1)
					{
						outfile << "Line(" << iter_1->src << "," << iter_1->dest << ") ";
						cout << "Line(" << iter_1->src << "," << iter_1->dest << ") ";
					}
					cout << endl;
					outfile << endl;

					outfile << "agent_2 repair_path:";
					for (auto iter_2 = agent_best_pi_2.begin(); iter_2 != agent_best_pi_2.end(); ++iter_2)
					{
						outfile << "Line(" << iter_2->src << "," << iter_2->dest << ") ";
						cout << "Line(" << iter_2->src << "," << iter_2->dest << ") ";
					}
					cout << endl;
					outfile << endl;

					outfile << "agent_3 repair_path:";
					for (auto iter_3 = agent_best_pi_3.begin(); iter_3 != agent_best_pi_3.end(); ++iter_3)
					{
						outfile << "Line(" << iter_3->src << "," << iter_3->dest << ") ";
						cout << "Line(" << iter_3->src << "," << iter_3->dest << ") ";
					}
					cout << endl;
					outfile << endl;

					outfile << "agent_4 repair_path:";
					for (auto iter_2 = agent_best_pi_4.begin(); iter_2 != agent_best_pi_4.end(); ++iter_2)
					{
						outfile << "Line(" << iter_2->src << "," << iter_2->dest << ") ";
						cout << "Line(" << iter_2->src << "," << iter_2->dest << ") ";
					}
					cout << endl;
					outfile << endl;

					outfile << "agent_5 repair_path:";
					for (auto iter_3 = agent_best_pi_5.begin(); iter_3 != agent_best_pi_5.end(); ++iter_3)
					{
						outfile << "Line(" << iter_3->src << "," << iter_3->dest << ") ";
						cout << "Line(" << iter_3->src << "," << iter_3->dest << ") ";
					}
					cout << endl;
					outfile << endl;

					outfile << "agent_6 repair_path:";
					for (auto iter_1 = agent_best_pi_6.begin(); iter_1 != agent_best_pi_6.end(); ++iter_1)
					{
						outfile << *iter_1 << "->";
						cout << *iter_1 << "->";
					}
					cout << endl;
					outfile << endl;

					outfile << "agent_7 repair_path:";
					for (auto iter_2 = agent_best_pi_7.begin(); iter_2 != agent_best_pi_7.end(); ++iter_2)
					{
						outfile << *iter_2 << "->";
						cout << *iter_2 << "->";
					}
					cout << endl;
					outfile << endl;

					outfile << "agent_8 repair_path:";
					for (auto iter_3 = agent_best_pi_8.begin(); iter_3 != agent_best_pi_8.end(); ++iter_3)
					{
						outfile << *iter_3 << "->";
						cout << *iter_3 << "->";
					}
					cout << endl;
					outfile << endl;

					outfile << "agent_9 repair_path:";
					for (auto iter_2 = agent_best_pi_9.begin(); iter_2 != agent_best_pi_9.end(); ++iter_2)
					{
						outfile << *iter_2 << "->";
						cout << *iter_2 << "->";
					}
					cout << endl;
					outfile << endl;

					outfile << "agent_10 repair_path:";
					for (auto iter_3 = agent_best_pi_10.begin(); iter_3 != agent_best_pi_10.end(); ++iter_3)
					{
						outfile << *iter_3 << "->";
						cout << *iter_3 << "->";
					}
					cout << endl;
					outfile << endl;

					outfile << "best_i =" << best_i + 1 << endl;
					outfile << "best_obj =" << best_obj << endl;
					outfile << "best_obj_RESCUE =" << best_obj_rescue << endl;
					cout << "best_obj =" << best_obj << endl;
					cout << "best_obj_RESCUE =" << best_obj_rescue << endl;
					outfile.close();

					char str2[20];
					sprintf_s(str2, "g%d_c%d_ACO", graph_index, AGENT_NUM);
					string tempstring2(str2);
					string filename2 = tempstring2 + "-object.txt";

					ofstream outfile2;
					outfile2.open(write_path + filename2, ofstream::app);
					//outfile2 << "best_obj = " << best_obj << endl;
					outfile2 << best_obj_rescue << endl;
					outfile2.close();

					char str3[20];
					sprintf_s(str3, "g%d_c%d_ACO", graph_index, AGENT_NUM);
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