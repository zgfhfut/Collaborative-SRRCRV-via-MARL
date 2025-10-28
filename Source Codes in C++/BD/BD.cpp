#include "random.h"			/*     Random Number Generator     */
#include "road.h"			/* Common class for road network processing */
#include <time.h>
#include <unordered_set>
#include<time.h>
#include<algorithm>
#include <stack>
#include<iostream>
#define maxpop 1	            /* Population size */

double seed;					/* Random seed */
double pcross;					/* Crossover probability */
double pmutate;					/* Mutation probability */
double a, b, c; // Dynamic reconstruction parameters
int lambda = 0.9;
/************************/
/* Enter basic chart information here */
/************************************************************************/

set<int> node_store; // Store road network nodes

set<int> demand_node_store;   // Store disaster points

set<Line> edge_store; // Store road network edges

set<Line> damaged_edge_store;   // Store damaged edges

map<int, double> max_allowed_dis_store_0; // Store maximum allowed distance

map<int, double> max_allowed_dis_store_1; // Store maximum allowed distance

map<int, double> max_allowed_dis_store_2; // Store maximum allowed distance

map<int, double> max_allowed_dis_store_3; // Store maximum allowed distance

map<int, double> max_allowed_dis_store_4; // Store maximum allowed distance

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

map<Line, int> damage_index_store; // Damaged segment - serial number lookup table

map<int, Line> index_damage_store; // Serial number - damaged segment lookup table

vector<int> cluster_store;
set<int> unexe_actions_set1; // Record set of unexecuted actions

map<int, bool> node_rescue;
map<double, set<Line>> all_time_state;
multimap<double, Line> time_state;
map<double, Line> times;
map<double, int> time_state_1; // Delivery time for all demand points
map<int, int> allagent;
map<int, int> agent_node; // Point division
/***********************************************************************/
void Initial()
{
	damage_index_store.clear();

	index_damage_store.clear();

	int index = 0;
	for (auto damage_iter = damaged_edge_store.begin(); damage_iter != damaged_edge_store.end(); ++damage_iter)
	{
		damage_index_store[*damage_iter] = index;
		index_damage_store[index] = *damage_iter;
		++index;
	}
}

void createGraph(Graph& g, string read_path, int agent_num) {
	node_store.clear();

	demand_node_store.clear();

	edge_store.clear();

	damaged_edge_store.clear();

	max_allowed_dis_store_0.clear();

	max_allowed_dis_store_1.clear();

	max_allowed_dis_store_2.clear();

	max_allowed_dis_store_3.clear();

	max_allowed_dis_store_4.clear();

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

	/**********************g.addEdge**********************/
	string add_edge = read_path + "add_edge.txt";
	ifstream infile1;
	infile1.open(add_edge);
	while (!infile1.eof()) {
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
	while (!infile2.eof()) {
		int node;
		double importance;
		string oneline;

		getline(infile2, oneline);
		stringstream ssin(oneline);

		ssin >> node;
		ssin >> importance;

		g.setImportance(node, importance);

		node_store.insert(node);

		if (importance != 0) demand_node_store.insert(node);
	}
	infile2.close();

	/**********************g.repairTime**********************/
	string set_repair = read_path + "set_repair.txt";
	ifstream infile3;
	infile3.open(set_repair);
	while (!infile3.eof()) {
		int src, dest;
		double repair_time;
		string oneline;
		getline(infile3, oneline);
		stringstream ssin(oneline);

		ssin >> src;
		ssin >> dest;
		ssin >> repair_time;

		g.setRepairTime(Line(src, dest), repair_time);
		damaged_edge_store.insert(Line(src, dest));
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
		g.setImportance(index, 0); // Set reserve point importance
	}


	/**********************RESCUE**********************/
	string set_rescue = read_path + "set_rescue.txt";
	ifstream infile6;
	infile6.open(set_rescue);

	while (!infile6.eof()) {
		int node;
		double rescue_time;
		string oneline;

		getline(infile6, oneline);
		stringstream ssin(oneline);

		ssin >> node;
		ssin >> rescue_time;
		g.setRescueTime(node, rescue_time); // Time spent delivering supplies at demand point

		//const_demand_node_store1.insert(node);

		node_rescue.insert(make_pair(node, false));

	}
	infile6.close();

	for (auto iter = damaged_edge_store.begin(); iter != damaged_edge_store.end(); ++iter)
	{
		g.setConnectionStatus(*iter, false);
	}
}

// Define individual and population
struct individual {
	vector<int> bincode;     // Encoding
	double fitness;          // Fitness value
	int endNum;              // Cutoff digit
	int max;
	int min;
	vector<vector<Line>> repairPath;  // Repair path
	vector<vector<int>> repaircode;  // Repair path
};

struct population {
	individual ind[maxpop];
	individual* ind_ptr;
};

struct population2 {
	individual ind[maxpop * 2];
	individual* ind_ptr;
};

struct population3 {
	individual ind[maxpop * 10];
	individual* ind_ptr;
};

void sleep(clock_t wait) {
	clock_t goal;
	goal = wait + clock();
	while (goal > clock());
}

// Initialize random function
void initialize() {
	srand((unsigned)time(NULL));
}

// Generate random integer between [a,b]
int getRandom(int lowerRound, int upperRound)
{
	int value;

	value = lowerRound + (int)(upperRound - lowerRound) * rand() / (RAND_MAX + 1);

	return value;
}

// Initialize road network
void inital_graph(Graph& g) {
	for (set<Line>::iterator damaged_iter = damaged_edge_store.begin(); damaged_iter != damaged_edge_store.end(); ++damaged_iter) {
		g.setConnectionStatus(*damaged_iter, false);
	}
	for (set<int>::iterator damaged_iter = demand_node_store.begin(); damaged_iter != demand_node_store.end(); ++damaged_iter)
		node_rescue[*damaged_iter] = false;
}

// Set crossover and mutation probabilities
void input() {
	pcross = 0.2;
	pmutate = 0.8;
	a = 0.2;
	b = 0.1;
	c = 1.2;
	lambda = 0.9;
}

// Establish correspondence between serial number and damaged edge
void initialMap(map<int, Line>& damagedIntLine, map<Line, int>& damagedLineInt, set<Line> damaged_edge_store) {
	int i = 0;
	for (set<Line>::iterator iter = damaged_edge_store.begin(); iter != damaged_edge_store.end(); iter++) {
		damagedIntLine.insert(pair<int, Line>(i, *iter));
		damagedLineInt.insert(pair<Line, int>(*iter, i));
		i++;
	}
}
void Initial_State(Graph g, set<int>& demand_nodes_set, int last_index)
{
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
	}
	//cout << "unexe_actions_set.size() = " << unexe_actions_set.size() << endl;
	//cout << "state_set.size() = " << state_set.size() << endl;
}
// Initialize population
void bininit(population* pop_ptr, vector<vector<int>>generate_list, int M) {
	int s;
	pop_ptr->ind_ptr = &(pop_ptr->ind[0]);
	for (s = 0; s < maxpop; s++) {
		pop_ptr->ind_ptr->bincode = generate_list[s];
		pop_ptr->ind_ptr->fitness = 0;
		pop_ptr->ind_ptr->repairPath.clear();

		pop_ptr->ind_ptr = &(pop_ptr->ind[s + 1]);
	}
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

bool findMinRepairCost(Graph g, set<int>& action_space, int& repair_action, int& next_dest, double& total_cost_repair, double& time, set<int> V_ACT, int agent) {
	double last_total_cost = time;
	double min_repair_cost;
	double temp_repair_cost;
	double temp_min_cost = 0, max_reward = 0;
	int temp_repair_action = 0;
	bool flag = false;
	vector<double> temp_travel_dis;
	vector<vector<int>> all_path_store;
	g.shortestPath2(next_dest, temp_travel_dis, all_path_store);
	//cout<< " actions_set.size() = "<< actions_set.size() << endl;
	vector<double> dis_to;
	g.shortestDis(next_dest, dis_to); // Find shortest distance from all nodes to point 0, store in dis_to_0
	map<int, double> max_allowed_dis_store;
	switch (agent)
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

	for (set<int>::iterator iter = action_space.begin(); iter != action_space.end(); ++iter) {
		//if (V_ACT.find(*iter) == V_ACT.end()){
			// Repair damaged edges in the path one by one, add to repair encoding, and calculate fitness value
			/*for (int k = 0; k < all_path_store[*iter].size() - 1; k++) {
				Line* tempLine = new Line(all_path_store[*iter][k], all_path_store[*iter][k + 1]);

				// If it's a damaged edge, repair it and add to repair encoding
				if (damagedLine.find(*tempLine) != damagedLine.end()) {
					damagedLine.erase(*tempLine);
					total_cost_repair += g.getRepairTime(*tempLine) + g.getWeight(*tempLine);
				}
				// If not damaged, pass directly
				else {
					total_cost_repair += g.getWeight(*tempLine);
				}
			}*/
		flag = true;
		temp_repair_action = *iter;
		double repair_travel_cost;
		double repair_line_cost;
		double repair_action_cost;
		repair_action_cost = temp_travel_dis[*iter] + g.getRescueTime(*iter);

		double temp_reward = g.getImportance(*iter) / temp_travel_dis[*iter];

		if (temp_reward >= max_reward) {
			temp_min_cost = repair_action_cost;
			max_reward = temp_reward;
			next_dest = *iter;
			repair_action = *iter;
			//}
		}
	}
	time = last_total_cost + temp_min_cost;
	action_space.erase(repair_action);

	//cout << "last_total_cost = " << last_total_cost << " " << time << endl;
	return flag;
}

void dpMethod(Graph g, int agent_num, individual& p) {

	unexe_actions_set1 = demand_node_store;

	double time_1 = 0, time_2 = 0, time_3 = 0, time = 0, time_4 = 0, time_5 = 0, time_6 = 0, time_7 = 0, time_8 = 0, time_9 = 0, time_10 = 0;

	bool exe_flag_6 = false, exe_flag_7 = false, exe_flag_8 = false, exe_flag_9 = false, exe_flag_10 = false; // Flag indicating whether previous action was executed

	bool act_flag_6 = false, act_flag_7 = false, act_flag_8 = false, act_flag_9 = false, act_flag_10 = false; // Flag indicating whether new action needs to be pushed to stack

	double total_cost_1 = 0, total_cost_2 = 0, total_cost_3 = 0, total_cost_4 = 0, total_cost_5 = 0;

	double total_cost_6 = 0, total_cost_7 = 0, total_cost_8 = 0, total_cost_9 = 0, total_cost_10 = 0;

	int last_dest_1 = 0, last_dest_2 = 1, last_dest_3 = 2, last_dest_4 = 3, last_dest_5 = 4, last_dest_6 = 0, last_dest_7 = 1, last_dest_8 = 2, last_dest_9 = 3, last_dest_10 = 4;

	int act_6 = 0, act_7 = 1, act_8 = 2, act_9 = 3, act_10 = 4;

	vector<vector<int>> repaircode(agent_num);
	// First initialize, remove already connected nodes from the graph

	for (set<Line>::iterator damaged_iter = damaged_edge_store.begin(); damaged_iter != damaged_edge_store.end(); ++damaged_iter) {
		g.setConnectionStatus(*damaged_iter, true);
	}
	for (int i = 0; i < agent_num; i++) {
		switch (i)
		{
		case 0: {
			exe_flag_6 = true;
			break;
		}
		case 1: {
			exe_flag_7 = true;
			break;
		}
		case 2: {
			exe_flag_8 = true;
			break;
		}
		case 3: {
			exe_flag_9 = true;
			break;
		}
		case 4: {
			exe_flag_10 = true;
			break;
		}
		default:
			break;
		}
	}
	// Dynamic programming, solve problem bottom-up, start from earliest subproblem, weighted_repair_cost stores optimal subproblem value, used in next calculation
	vector<int> V_ACT_r;
	map<int, int> action_agent_r; // Record action corresponding repair team
	map<int, double> action_fine_r; // Record corresponding action execution time
	set<int> v_r;
	while (!unexe_actions_set1.empty()) {
		int exe_act = 0; // Record action to be executed in buffer
		act_flag_6 = act_flag_7 = act_flag_8 = act_flag_9 = act_flag_10 = false;
		bool quit_flag = false;

		if (exe_flag_6)
		{
			quit_flag = findMinRepairCost(g, unexe_actions_set1, act_6, last_dest_6, total_cost_6, time_6, v_r, 0); // demand_available_nodes is the demand node connected this time
			v_r.insert(act_6);
			action_agent_r[act_6] = 1; // Set repair team executing action act_1

			action_fine_r[act_6] = time_6; // Store time after executing act_1
			exe_flag_6 = false;
			act_flag_6 = true;
			repaircode[0].push_back(act_6);
		}
		if (exe_flag_7)
		{
			quit_flag = findMinRepairCost(g, unexe_actions_set1, act_7, last_dest_7, total_cost_7, time_7, v_r, 1); // demand_available_nodes is the demand node connected this time
			v_r.insert(act_7);
			action_agent_r[act_7] = 2; // Set repair team executing action act_1=1

			action_fine_r[act_7] = time_7; // Store time after executing act_1
			exe_flag_7 = false;
			act_flag_7 = true;
			repaircode[1].push_back(act_7);
		}
		if (exe_flag_8)
		{
			quit_flag = findMinRepairCost(g, unexe_actions_set1, act_8, last_dest_8, total_cost_8, time_8, v_r, 2); // demand_available_nodes is the demand node connected this time
			v_r.insert(act_8);
			action_agent_r[act_8] = 3; // Set repair team executing action act_1=1

			action_fine_r[act_8] = time_8; // Store time after executing act_1			
			exe_flag_8 = false;
			act_flag_8 = true;
			repaircode[2].push_back(act_8);
		}
		if (exe_flag_9)
		{
			quit_flag = findMinRepairCost(g, unexe_actions_set1, act_9, last_dest_9, total_cost_9, time_9, v_r, 3); // demand_available_nodes is the demand node connected this time
			v_r.insert(act_9);
			action_agent_r[act_9] = 4; // Set repair team executing action act_1=1

			action_fine_r[act_9] = time_9; // Store time after executing act_1
			exe_flag_9 = false;
			act_flag_9 = true;
			repaircode[3].push_back(act_9);
		}
		if (exe_flag_10)
		{
			quit_flag = findMinRepairCost(g, unexe_actions_set1, act_10, last_dest_10, total_cost_10, time_10, v_r, 4); // demand_available_nodes is the demand node connected this time
			v_r.insert(act_10);
			action_agent_r[act_10] = 5; // Set repair team executing action act_1=1

			action_fine_r[act_10] = time_10; // Store time after executing act_1			
			exe_flag_10 = false;
			act_flag_10 = true;
			repaircode[4].push_back(act_10);
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
			int min_index = V_ACT_r.size(); // Index of minimum time consumption
			exe_act = V_ACT_r[min_index - 1]; // Currently least time-consuming action, take one action each time
			auto it = V_ACT_r.begin() + min_index - 1;
			V_ACT_r.erase(it);
		}
		else if (V_ACT_r.size() == 1)
		{
			exe_act = V_ACT_r[0];
			V_ACT_r.clear();
		}
		p.bincode.push_back(exe_act);
		//cout << "unexe_actions_set1 = " << unexe_actions_set1.size() << " exe_act  = " << exe_act <<"action_agent_r[exe_act] = "<< action_agent_r[exe_act] << endl;
		cout << "unexe_actions_set1 = " << unexe_actions_set1.size() << "exe_act = " << exe_act << "V_ACT_r.size() = " << V_ACT_r.size() << endl;
		switch (action_agent_r[exe_act])
		{
		case 1: {
			total_cost_6 = time_6;
			exe_flag_6 = true;
			break;
		}
		case 2: {
			;
			total_cost_7 = time_7;
			exe_flag_7 = true;
			break;
		}
		case 3: {
			total_cost_8 = time_8;
			exe_flag_8 = true;
			break;
		}
		case 4: {
			total_cost_9 = time_9;
			exe_flag_9 = true;
			break;
		}
		case 5: {
			total_cost_10 = time_10;
			exe_flag_10 = true;
			break;
		}
		default:break;
		}
		if (unexe_actions_set1.empty()) {
			//cout << "Transportation ended!" << endl;
			exe_flag_6 = exe_flag_7 = exe_flag_8 = exe_flag_9 = exe_flag_10 = false;
		}
	}
	p.repaircode = repaircode;
	/*for (auto iter = repaircode[0].begin(); iter != repaircode[0].end(); ++iter)
	{
		cout << *iter<< " ";
	}
	cout << p.repaircode[0].size() + p.repaircode[1].size() + p.repaircode[2].size() << endl;
	cout << endl;*/
}

void bincode(individual& p, int agent_num) {
	p.repaircode.clear();
	set<int> visited;
	vector<vector<int>> repaircode(agent_num);
	cout << "Length=" << repaircode[0].size() << repaircode[1].size() << repaircode[2].size() << repaircode[3].size() << repaircode[4].size() << endl;
	//cout << " after =  " << p.bincode.size() << endl;
	for (int i = 0; i < p.bincode.size(); i++) {

		int choose_act = p.bincode[i];

		if (visited.find(choose_act) != visited.end()) cout << "error1!" << endl;

		repaircode[agent_node[choose_act] - 1].push_back(choose_act);
		//else
		//	PI[agent_index].push_back(choose_act);
		//cout << damaged[i] << "agent = " << damaged[i + M] << endl;

		visited.insert(choose_act);
		cout << choose_act << " ";
	}
	p.repaircode = repaircode;
	cout << endl;
}

void bincode_rescue(individual& p, int agent_num) {
	p.repaircode.clear();
	set<int> visited;
	vector<vector<int>> repaircode(agent_num);
	//cout <<" before =  "<< p.bincode.size();
	for (int i = 0; i < p.bincode.size(); i++) {

		int choose_act = p.bincode[i];

		if (visited.find(choose_act) != visited.end()) cout << "error1!" << endl;

		repaircode[agent_node[choose_act]].push_back(choose_act);

		visited.insert(choose_act);
		//cout << p.bincode[i] << " ";
	}
	p.repaircode = repaircode;
	//cout << endl;
}

int Maxnum(double num[], int agent_num) {
	int k = -1;
	double max = INF;
	for (int i = 0; i < agent_num; i++) {
		if (num[i] < max && num[i] != 0) {
			max = num[i];
			k = i;
		}
	}
	if (max == INF) cout << " Error! " << num[0] << " " << num[1] << " " << num[2] << endl;
	return k;
}

// Repair strategy
double decode_and_function1(individual& p, Graph& g, int agent_num, bool flag) {
	// Before analyzing results, reset damaged edges to initial state
	for (set<Line>::iterator damaged_iter = damaged_edge_store.begin(); damaged_iter != damaged_edge_store.end(); ++damaged_iter) {
		g.setConnectionStatus(*damaged_iter, false);
	}
	vector<vector<Line>> repairPath(agent_num);
	vector<vector<int>> PI = p.repaircode;
	double object = 0;      // Fitness value
	vector<double> maxmin;
	int index1, index2;
	time_state.clear();
	allagent.clear();
	if (agent_num == 3) cout << "Repair team encoding length：" << PI[0].size() + PI[1].size() + PI[2].size() << endl;
	else cout << "Repair team encoding length：" << PI[0].size() + PI[1].size() + PI[2].size() + PI[3].size() + PI[4].size() << endl;

	bool exe_flag_1 = false, exe_flag_2 = false, exe_flag_3 = false, exe_flag_4 = false, exe_flag_5 = false; // Flag indicating whether previous action was executed

	double total_cost_1 = 0, total_cost_2 = 0, total_cost_3 = 0, total_cost_4 = 0, total_cost_5 = 0;

	bool act_flag_1 = false, act_flag_2 = false, act_flag_3 = false, act_flag_4 = false, act_flag_5 = false;

	int index_1 = 0, index_2 = 0, index_3 = 0, index_4 = 0, index_5 = 0;

	int act_1 = 0, act_2 = 0, act_3 = 0, act_4 = 0, act_5 = 0;

	int last_index_1 = 0, last_index_2 = 1, last_index_3 = 2, last_index_4 = 3, last_index_5 = 4;
	for (int i = 0; i < agent_num; i++) {
		switch (i) {
		case 0: {
			exe_flag_1 = true;
			break;
		}
		case 1: {
			exe_flag_2 = true;
			break;
		}
		case 2: {
			exe_flag_3 = true;
			break;
		}
		case 3: {
			exe_flag_4 = true;
			break;
		}
		case 4: {
			exe_flag_5 = true;
			break;
		}
		default: break;
		}
	}
	vector<int> V_ACT; // Buffer storing actions to be executed
	int exe_action; // Record action popped from stack to be executed
	map<int, int> action_agent; // Record action corresponding repair team
	map<int, double> action_fine; // Record corresponding action execution time
	double weighted_total_repair_cost = 0; // Weighted time cost
	double weighted_total_dis = 0;	// Weighted calculation of repair distance

	set<int> rest_demand_nodes = demand_node_store;   // Initialize remaining demand nodes
	set<Line> damagedLine = damaged_edge_store;  // Damaged edges

	vector<double> dis_to_0;
	vector<double> dis_to_1;
	vector<double> dis_to_2;
	vector<double> dis_to_3;
	vector<double> dis_to_4;

	vector<int> repairCode;   // Repair encoding
	int number_1 = 0, number_2 = 0, number_3 = 0, number_4 = 0, number_5 = 0;          // Record cutoff digit, i.e., number of edges repair team needs to repair
	p.repairPath.clear();          // Clear repair path in individual
	map<Line, double> actLine1, actLine2, actLine3, actLine4, actLine5;

	while (index_1 < PI[0].size() || index_2 < PI[1].size() || index_3 < PI[2].size() || index_4 < PI[3].size() || index_5 < PI[4].size() || !V_ACT.empty()) {
		act_flag_1 = act_flag_2 = act_flag_3 = act_flag_4 = act_flag_5 = false;

		// Start decoding, only after previous action is executed can next action start
		if (exe_flag_1 && index_1 < PI[0].size()) // Only after previous action is executed can next action start
		{
			exe_flag_1 = false;
			act_flag_1 = true;
			// First check if encoding has appeared in repairCode, if yes, it means already repaired (might have been repaired earlier during repair process)
			act_1 = PI[0][index_1];
			vector<double> dis_store;
			vector<vector<int>> all_path_store1;
			g.shortestDis(last_index_1, dis_store);

			// If unreachable, select nearest path (including damaged edges)
			if (dis_store[act_1] == INF) {
				vector<double> dis_store1;
				vector<vector<int>> all_path_store;
				g.shortestPath2(last_index_1, dis_store1, all_path_store);
				// Repair damaged edges in the path one by one, add to repair encoding, and calculate fitness value
				for (int k = 0; k < all_path_store[act_1].size() - 1; k++) {
					Line* tempLine = new Line(all_path_store[act_1][k], all_path_store[act_1][k + 1]);

					// If damaged edge, repair and add to repair encoding
					if (damagedLine.find(*tempLine) != damagedLine.end()) {
						//damagedLine.erase(*tempLine);
						total_cost_1 += g.getRepairTime(*tempLine) + g.getWeight(*tempLine);
						actLine1.insert(make_pair(*tempLine, total_cost_1));
					}
					// If not damaged, pass directly
					else {
						total_cost_1 += g.getWeight(*tempLine);
						actLine1.insert(make_pair(*tempLine, total_cost_1));
					}
				}
			}
			// If reachable, go directly
			else {
				total_cost_1 += dis_store[act_1];
			}
			//cout << "act_1 = " << act_1 << endl;
			action_agent[act_1] = 1;
			action_fine[act_1] = total_cost_1;
		}
		if (exe_flag_2 && index_2 < PI[1].size()) // Only after previous action is executed can next action start
		{
			exe_flag_2 = false;
			act_flag_2 = true;
			// First check if encoding has appeared in repairCode, if yes, it means already repaired (might have been repaired earlier during repair process)
			number_2++; // Record encoding length
			act_2 = PI[1][index_2];
			vector<double> dis_store;
			vector<vector<int>> all_path_store1;
			g.shortestDis(last_index_2, dis_store);

			// If unreachable, select nearest path (including damaged edges)
			if (dis_store[act_2] == INF) {
				vector<double> dis_store1;
				vector<vector<int>> all_path_store;
				g.shortestPath2(last_index_2, dis_store1, all_path_store);

				// Repair damaged edges in the path one by one, add to repair encoding, and calculate fitness value
				for (int k = 0; k < all_path_store[act_2].size() - 1; k++) {
					Line* tempLine = new Line(all_path_store[act_2][k], all_path_store[act_2][k + 1]);

					// If damaged edge, repair and add to repair encoding
					if (damagedLine.find(*tempLine) != damagedLine.end()) {
						total_cost_2 += g.getRepairTime(*tempLine) + g.getWeight(*tempLine);
						actLine2.insert(make_pair(*tempLine, total_cost_2));
					}
					// If not damaged, pass directly
					else {
						total_cost_2 += g.getWeight(*tempLine);
						actLine2.insert(make_pair(*tempLine, total_cost_2));
					}
				}
			}
			// If either point is reachable, go repair directly
			else {
				total_cost_2 += dis_store[act_2];
			}
			action_agent[act_2] = 2;
			action_fine[act_2] = total_cost_2;
			//cout << "act_2 = " << act_2 << endl;
			//cout << "total_cost_2 = " << total_cost_2 << endl;
		}

		if (exe_flag_3 && index_3 < PI[2].size()) // Only after previous action is executed can next action start
		{
			exe_flag_3 = false;
			act_flag_3 = true;
			// First check if encoding has appeared in repairCode, if yes, it means already repaired (might have been repaired earlier during repair process)
			number_3++; // Record encoding length
			act_3 = PI[2][index_3];
			vector<double> dis_store;
			vector<vector<int>> all_path_store1;
			g.shortestDis(last_index_3, dis_store);

			// If unreachable, select nearest path (including damaged edges)
			if (dis_store[act_3] == INF) {
				vector<double> dis_store1;
				vector<vector<int>> all_path_store;
				g.shortestPath2(last_index_3, dis_store1, all_path_store);

				// Repair damaged edges in the path one by one, add to repair encoding, and calculate fitness value
				for (int k = 0; k < all_path_store[act_3].size() - 1; k++) {
					Line* tempLine = new Line(all_path_store[act_3][k], all_path_store[act_3][k + 1]);

					// If damaged edge, repair and add to repair encoding
					if (damagedLine.find(*tempLine) != damagedLine.end()) {
						total_cost_3 += g.getRepairTime(*tempLine) + g.getWeight(*tempLine);
						actLine3.insert(make_pair(*tempLine, total_cost_3));
					}
					// If not damaged, pass directly
					else {
						total_cost_3 += g.getWeight(*tempLine);
					}
				}
			}
			// If either point is reachable, go repair directly
			else {
				total_cost_3 += dis_store[act_3];
			}
			action_agent[act_3] = 3;
			action_fine[act_3] = total_cost_3;
			//cout << "act_3 = " << act_3 << endl;
			//cout << "total_cost_2 = " << total_cost_2 << endl;
		}

		if (exe_flag_4 && index_4 < PI[3].size()) // Only after previous action is executed can next action start
		{
			exe_flag_4 = false;
			act_flag_4 = true;
			// First check if encoding has appeared in repairCode, if yes, it means already repaired (might have been repaired earlier during repair process)
			number_4++; // Record encoding length
			act_4 = PI[3][index_4];
			vector<double> dis_store;
			vector<vector<int>> all_path_store1;
			g.shortestDis(last_index_4, dis_store);

			// If unreachable, select nearest path (including damaged edges)
			if (dis_store[act_4] == INF) {
				vector<double> dis_store1;
				vector<vector<int>> all_path_store;
				g.shortestPath2(last_index_4, dis_store1, all_path_store);

				// Repair damaged edges in the path one by one, add to repair encoding, and calculate fitness value
				for (int k = 0; k < all_path_store[act_4].size() - 1; k++) {
					Line* tempLine = new Line(all_path_store[act_4][k], all_path_store[act_4][k + 1]);

					// If damaged edge, repair and add to repair encoding
					if (damagedLine.find(*tempLine) != damagedLine.end()) {
						total_cost_4 += g.getRepairTime(*tempLine) + g.getWeight(*tempLine);
						actLine4.insert(make_pair(*tempLine, total_cost_4));
					}
					// If not damaged, pass directly
					else {
						total_cost_4 += g.getWeight(*tempLine);
					}
				}
			}
			// If either point is reachable, go repair directly
			else {
				total_cost_4 += dis_store[act_4];
			}
			action_agent[act_4] = 4;
			action_fine[act_4] = total_cost_4;
			//cout << "act_4 = " << act_4 << endl;
			//cout << "total_cost_2 = " << total_cost_2 << endl;
		}

		if (exe_flag_5 && index_5 < PI[4].size()) // Only after previous action is executed can next action start
		{
			exe_flag_5 = false;
			act_flag_5 = true;
			// First check if encoding has appeared in repairCode, if yes, it means already repaired (might have been repaired earlier during repair process)
			number_5++; // Record encoding length
			act_5 = PI[4][index_5];
			vector<double> dis_store;
			vector<vector<int>> all_path_store1;
			g.shortestDis(last_index_5, dis_store);

			// If unreachable, select nearest path (including damaged edges)
			if (dis_store[act_5] == INF) {
				vector<double> dis_store1;
				vector<vector<int>> all_path_store;
				g.shortestPath2(last_index_5, dis_store1, all_path_store);

				// Repair damaged edges in the path one by one, add to repair encoding, and calculate fitness value
				for (int k = 0; k < all_path_store[act_5].size() - 1; k++) {
					Line* tempLine = new Line(all_path_store[act_5][k], all_path_store[act_5][k + 1]);

					// If damaged edge, repair and add to repair encoding
					if (damagedLine.find(*tempLine) != damagedLine.end()) {
						total_cost_5 += g.getRepairTime(*tempLine) + g.getWeight(*tempLine);
						actLine5.insert(make_pair(*tempLine, total_cost_5));
					}
					// If not damaged, pass directly
					else {
						total_cost_5 += g.getWeight(*tempLine);
					}
				}
			}
			// If either point is reachable, go repair directly
			else {
				total_cost_5 += dis_store[act_5];
			}
			action_agent[act_5] = 5;
			action_fine[act_5] = total_cost_5;
			//cout << "act_5 = " << act_5 << endl;
			//cout << "total_cost_2 = " << total_cost_2 << endl;
		}

		if (act_flag_1) V_ACT.push_back(act_1);

		if (act_flag_2) V_ACT.push_back(act_2);

		if (act_flag_3) V_ACT.push_back(act_3);

		if (act_flag_4) V_ACT.push_back(act_4);

		if (act_flag_5) V_ACT.push_back(act_5);

		if (V_ACT.size() > 1)
		{
			Bubble1(V_ACT, action_fine);
			int min_index = V_ACT.size(); // Index of minimum time consumption
			exe_action = V_ACT[min_index - 1]; // Currently least time-consuming action, take one action each time
			auto it = V_ACT.begin() + min_index - 1;
			V_ACT.erase(it);
		}
		else if (V_ACT.size() == 1)
		{
			exe_action = V_ACT[0];
			V_ACT.clear();
		}
		if (act_flag_1 == false && act_flag_2 == false && act_flag_3 == false && act_flag_4 == false && act_flag_5 == false && V_ACT.size() == 0) {
			exe_flag_1 = exe_flag_2 = exe_flag_3 = true;
		}
		//cout << "exe_action = " << exe_action <<" -- " << action_agent[exe_action] <<";" << action_fine[exe_action] <<endl;
		agent_node.insert(make_pair(exe_action, action_agent[exe_action]));
		// Set flag corresponding to executed action to 1
		switch (action_agent[exe_action]) {
		case 1: {
			//cout << "11111111111" << endl;
			exe_flag_1 = true;
			weighted_total_repair_cost += g.getImportance(exe_action) * action_fine[exe_action];
			rest_demand_nodes.erase(exe_action);
			for (auto iter = actLine1.begin(); iter != actLine1.end(); ++iter) {
				if (damagedLine.find(iter->first) != damagedLine.end()) {
					g.setConnectionStatus(iter->first, true);
					time_state.insert(make_pair(iter->second, iter->first));
					damagedLine.erase(iter->first);
					agent_best_pi_1.push_back(iter->first);
					repairPath[0].push_back(iter->first);
					repairCode.push_back(damage_index_store[iter->first]);
				}
			}
			index_1++;
			last_index_1 = act_1;
			break;
		}
		case 2: {
			//cout << "22222222222" << endl;
			exe_flag_2 = true;
			weighted_total_repair_cost += g.getImportance(exe_action) * action_fine[exe_action];
			rest_demand_nodes.erase(exe_action);
			for (auto iter = actLine2.begin(); iter != actLine2.end(); ++iter) {
				if (damagedLine.find(iter->first) != damagedLine.end()) {
					g.setConnectionStatus(iter->first, true);
					time_state.insert(make_pair(iter->second, iter->first));
					damagedLine.erase(iter->first);
					agent_best_pi_2.push_back(iter->first);
					repairPath[1].push_back(iter->first);
					repairCode.push_back(damage_index_store[iter->first]);
				}
			}
			index_2++;
			last_index_2 = act_2;
			break;
		}
		case 3: {
			//cout << "33333333333333" << endl;
			exe_flag_3 = true;
			weighted_total_repair_cost += g.getImportance(exe_action) * action_fine[exe_action];
			rest_demand_nodes.erase(exe_action);
			for (auto iter = actLine3.begin(); iter != actLine3.end(); ++iter) {
				if (damagedLine.find(iter->first) != damagedLine.end()) {
					g.setConnectionStatus(iter->first, true);
					time_state.insert(make_pair(iter->second, iter->first));
					damagedLine.erase(iter->first);
					agent_best_pi_3.push_back(iter->first);
					repairPath[2].push_back(iter->first);
					repairCode.push_back(damage_index_store[iter->first]);
				}
			}
			index_3++;
			last_index_3 = act_3;
			break;
		}
		case 4: {
			//cout << "44444444444444" << endl;
			exe_flag_4 = true;
			weighted_total_repair_cost += g.getImportance(exe_action) * action_fine[exe_action];
			rest_demand_nodes.erase(exe_action);
			for (auto iter = actLine4.begin(); iter != actLine4.end(); ++iter) {
				if (damagedLine.find(iter->first) != damagedLine.end()) {
					g.setConnectionStatus(iter->first, true);
					time_state.insert(make_pair(iter->second, iter->first));
					damagedLine.erase(iter->first);
					agent_best_pi_4.push_back(iter->first);
					repairPath[3].push_back(iter->first);
					repairCode.push_back(damage_index_store[iter->first]);
				}
			}
			index_4++;
			last_index_4 = act_4;
			break;
		}
		case 5: {
			//cout << "55555555555555555" << endl;
			exe_flag_5 = true;
			weighted_total_repair_cost += g.getImportance(exe_action) * action_fine[exe_action];
			rest_demand_nodes.erase(exe_action);
			for (auto iter = actLine5.begin(); iter != actLine5.end(); ++iter) {
				if (damagedLine.find(iter->first) != damagedLine.end()) {
					g.setConnectionStatus(iter->first, true);
					time_state.insert(make_pair(iter->second, iter->first));
					damagedLine.erase(iter->first);
					agent_best_pi_5.push_back(iter->first);
					repairPath[4].push_back(iter->first);
					repairCode.push_back(damage_index_store[iter->first]);
				}
			}
			index_5++;
			last_index_5 = act_5;
			break;
		}
		default:break;
		}
		if (rest_demand_nodes.size() == 0) break;
		//cout << "rest_demand_nodes.size() = " << rest_demand_nodes.size() << " "<< V_ACT.size() << endl;
		//cout << index_1 << " " << index_2 << " " << index_3 << "size_error!" << endl;
		vector<double> dis_to_0;
		vector<double> dis_to_1;
		vector<double> dis_to_2;
		vector<double> dis_to_3;
		vector<double> dis_to_4;
		g.shortestDis(0, dis_to_0);
		g.shortestDis(1, dis_to_1);
		g.shortestDis(2, dis_to_2);
		g.shortestDis(3, dis_to_3);
		g.shortestDis(4, dis_to_4);
		double min_dis = INF;
		double nums[5];
		for (int index = 0; index < agent_num; ++index) {
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
			nums[index] = dis_array[exe_action];
			if (dis_array[exe_action] < min_dis) {
				min_dis = dis_array[exe_action];
			}
		}
		weighted_total_dis += g.getImportance(exe_action) * min_dis;
		if (min_dis == INF) cout << "object_error!!!" << exe_action << endl;

		int k = Maxnum(nums, agent_num);
		allagent.insert(make_pair(exe_action, k));
	}
	if (rest_demand_nodes.size() != 0) cout << "Repair not finished！！！！！！" << endl;

	// Calculate objective function
	double lambda = 0.9;       // Weighting factor λ
	object = weighted_total_dis * lambda + weighted_total_repair_cost * (1 - lambda);
	p.repairPath = repairPath;
	p.fitness = object;
	cout << "object = " << object << endl;
	return object;
	//cout << " after =  " << p.bincode.size();
}

double CalObj_rescue(individual& p, Graph& g, int agent_num, bool flag)
{
	inital_graph(g);
	vector<vector<int>> PI = p.repaircode;
	vector<int> p1, p2, p3, p4, p5;
	time_state_1.clear();
	/*p1 = PI[0], p2 = PI[1], p3 = PI[2];
	if (agent_num == 5) {
		p4 = PI[3], p5 = PI[4];
	}*/
	unexe_actions_set1 = demand_node_store;
	set<int> rest_demand_nodes = demand_node_store; // Record remaining isolated demand points

	for (auto iter = unexe_actions_set1.begin(); iter != unexe_actions_set1.end(); ++iter)
	{
		switch (agent_node[*iter]) {
		case 1: {
			p1.push_back(*iter);
			break;
		}
		case 2: {
			p2.push_back(*iter);
			break;
		}
		case 3: {
			p3.push_back(*iter);
			break;
		}
		case 4: {
			p4.push_back(*iter);
			break;
		}
		case 5: {
			p5.push_back(*iter);
			break;
		}
		default:break;
		}
	}
	double weighted_repair_cost = 0;
	vector<double> maxmin;
	int index1, index2;

	bool exe_flag_1 = false, exe_flag_2 = false, exe_flag_3 = false, exe_flag_4 = false, exe_flag_5 = false; // Flag indicating whether previous action was executed

	double total_cost_1 = 0, total_cost_2 = 0, total_cost_3 = 0, total_cost_4 = 0, total_cost_5 = 0;

	bool act_flag_1 = false, act_flag_2 = false, act_flag_3 = false, act_flag_4 = false, act_flag_5 = false;

	int index_1 = 0, index_2 = 0, index_3 = 0, index_4 = 0, index_5 = 0;

	int act_1 = 0, act_2 = 0, act_3 = 0, act_4 = 0, act_5 = 0;

	int last_dest_1 = 0, last_dest_2 = 1, last_dest_3 = 2, last_dest_4 = 3, last_dest_5 = 4;

	for (int i = 0; i < agent_num; i++) {
		switch (i) {
		case 0: {
			exe_flag_1 = true;
			break;
		}
		case 1: {
			exe_flag_2 = true;
			break;
		}
		case 2: {
			exe_flag_3 = true;
			break;
		}
		case 3: {
			exe_flag_4 = true;
			break;
		}
		case 4: {
			exe_flag_5 = true;
			break;
		}
		default: break;
		}
	}
	vector<int> V_ACT; // Buffer storing actions to be executed
	int exe_action = 0; // Record action popped from stack to be executed
	map<int, int> action_agent; // Record action corresponding repair team
	map<int, double> action_fine; // Record corresponding action execution time
	cout << "Transport team encoding length：" << p1.size() + p2.size() + p3.size() + p4.size() + p5.size() << endl;

	while (index_1 < p1.size() || index_2 < p2.size() || index_3 < p2.size() || index_4 < p5.size() || index_5 < p5.size() || !V_ACT.empty()) {
		act_flag_1 = act_flag_2 = act_flag_3 = act_flag_4 = act_flag_5 = false;
		//cout << index_1 << " " << index_2 << " " << index_3 << " " << index_4 << " " << index_5 << " " << !V_ACT.size() << endl;
		if (exe_flag_1 && index_1 < (int)p1.size()) // Only after previous action is executed can next action start
		{
			act_1 = p1[index_1];
			//cout << "act_1 = " << act_1 << endl;
			vector<double> dis_array;
			vector<double> dis_array1;
			vector<Line> s;
			for (auto iter = time_state.begin(); iter != time_state.end(); iter++) {
				if (total_cost_1 >= iter->first) {
					g.setConnectionStatus(iter->second, true);
					s.push_back(iter->second);
				}
				else break;
			}
			g.shortestDis(last_dest_1, dis_array);
			//cout << "iter->second = " << s.size() << endl;
			for (auto iter = s.begin(); iter != s.end(); iter++) g.setConnectionStatus(*iter, false);
			double travel_cost = dis_array[act_1]; // End position of last action to start position of next action

			if (travel_cost != INF) {
				act_flag_1 = true;  // New action needs to be pushed to stack
				//cout << "right!" << endl;
				index_1++;
				total_cost_1 += travel_cost + g.getRescueTime(act_1);
				last_dest_1 = act_1; // End position of this action
				action_agent[act_1] = 1; // This action completed by repair team 1
				action_fine[act_1] = total_cost_1; // Record time consumed by this action
				exe_flag_1 = false; // Action executed
				agent_best_pi_6.push_back(act_1);
			}
			else {
				//cout << "1 Wait in place！" << total_cost_1 << endl;
				total_cost_1 += 5;
			}
		}
		if (exe_flag_2 && index_2 < (int)p2.size())
		{
			act_2 = p2[index_2];
			//cout << "act_2 = " << act_2 << endl;
			vector<double> dis_array;
			vector<double> dis_array1;
			vector<Line> s;
			for (auto iter = time_state.begin(); iter != time_state.end(); iter++) {
				if (total_cost_2 >= iter->first) {
					g.setConnectionStatus(iter->second, true);
					s.push_back(iter->second);
				}
				else break;
			}
			g.shortestDis(last_dest_2, dis_array);
			//cout << "iter->second = " << s.size() << endl;
			for (auto iter = s.begin(); iter != s.end(); iter++) g.setConnectionStatus(*iter, false);
			double travel_cost = dis_array[act_2]; // End position of last action to start position of next action

			if (travel_cost != INF) {
				act_flag_2 = true;
				//cout << "right!" << endl;
				index_2++;
				total_cost_2 += travel_cost + g.getRescueTime(act_2);
				last_dest_2 = act_2;
				action_agent[act_2] = 2;
				action_fine[act_2] = total_cost_2;
				exe_flag_2 = false;
				agent_best_pi_7.push_back(act_2);
			}
			else {
				//cout << "2 Wait in place！" << total_cost_2 << endl;
				total_cost_2 += 5;
			}
		}

		if (exe_flag_3 && index_3 < (int)p3.size())
		{
			act_3 = p3[index_3];
			//cout << "act_3 = " << act_3 << endl;
			vector<double> dis_array;
			vector<Line> s;
			for (auto iter = time_state.begin(); iter != time_state.end(); iter++) {
				if (total_cost_3 >= iter->first) {
					g.setConnectionStatus(iter->second, true);
					s.push_back(iter->second);
				}
				else break;
			}
			//cout << "iter->second = " << s.size() << endl;
			g.shortestDis(last_dest_3, dis_array);
			for (auto iter = s.begin(); iter != s.end(); iter++) g.setConnectionStatus(*iter, false);
			double travel_cost = dis_array[act_3]; // End position of last action to start position of next action

			if (travel_cost != INF) {
				act_flag_3 = true;
				//cout << "right!" << endl;
				index_3++;
				total_cost_3 += travel_cost + g.getRescueTime(act_3);
				last_dest_3 = act_3;
				action_agent[act_3] = 3;
				action_fine[act_3] = total_cost_3;
				exe_flag_3 = false;
				agent_best_pi_8.push_back(act_3);
			}
			else {
				//cout << "3 Wait in place！" << total_cost_3 << endl;
				total_cost_3 += 5;
			}
		}

		if (exe_flag_4 && index_4 < (int)p4.size())
		{
			act_4 = p4[index_4];
			//cout << "act_2 = " << act_2 << endl;
			vector<double> dis_array;
			vector<double> dis_array2;
			vector<Line> s;
			for (auto iter = time_state.begin(); iter != time_state.end(); iter++) {
				if (total_cost_4 >= iter->first) {
					g.setConnectionStatus(iter->second, true);
					s.push_back(iter->second);
				}
				else break;
			}
			//cout << "iter->second = " << s.size() << endl;
			g.shortestDis(last_dest_4, dis_array);
			for (auto iter = s.begin(); iter != s.end(); iter++) g.setConnectionStatus(*iter, false);
			double travel_cost = dis_array[act_4]; // End position of last action to start position of next action

			if (travel_cost != INF) {
				act_flag_4 = true;
				//cout << "right!" << endl;
				index_4++;
				total_cost_4 += travel_cost + g.getRescueTime(act_4);
				last_dest_4 = act_4;
				action_agent[act_4] = 4;
				action_fine[act_4] = total_cost_4;
				exe_flag_4 = false;
				agent_best_pi_9.push_back(act_4);
			}
			else {
				//cout << "4 Wait in place！" << total_cost_4 << endl;
				total_cost_4 += 5;
			}
		}

		if (exe_flag_5 && index_5 < (int)p5.size())
		{
			act_5 = p5[index_5];
			//cout << "act_3 = " << act_3 << endl;
			vector<double> dis_array;
			vector<Line> s;
			for (auto iter = time_state.begin(); iter != time_state.end(); iter++) {
				if (total_cost_5 >= iter->first) {
					g.setConnectionStatus(iter->second, true);
					s.push_back(iter->second);
				}
				else break;
			}
			//cout << "iter->second = " << s.size() << endl;
			g.shortestDis(last_dest_5, dis_array);
			for (auto iter = s.begin(); iter != s.end(); iter++) g.setConnectionStatus(*iter, false);
			double travel_cost = dis_array[act_5]; // End position of last action to start position of next action

			if (travel_cost != INF) {
				act_flag_5 = true;
				//cout << "right!" << endl;
				index_5++;
				total_cost_5 += travel_cost + g.getRescueTime(act_5);
				last_dest_5 = act_5;
				action_agent[act_5] = 5;
				action_fine[act_5] = total_cost_5;
				exe_flag_5 = false;
				agent_best_pi_10.push_back(act_5);
			}
			else {
				//cout << "5 Wait in place！" << total_cost_5 << endl;
				total_cost_5 += 5;
			}
		}
		if (act_flag_1) V_ACT.push_back(act_1); // Buffer storing actions to be executed, action1 pushed to stack

		if (act_flag_2) V_ACT.push_back(act_2);

		if (act_flag_3) V_ACT.push_back(act_3);

		if (act_flag_4) V_ACT.push_back(act_4);

		if (act_flag_5) V_ACT.push_back(act_5);
		/************************************************************************/
		if (V_ACT.size() > 1)
		{
			Bubble1(V_ACT, action_fine);
			int min_index = V_ACT.size(); // Index of minimum time consumption
			exe_action = V_ACT[min_index - 1]; // Currently least time-consuming action, take one action each time
			auto it = V_ACT.begin() + min_index - 1;
			V_ACT.erase(it);
		}
		else if (V_ACT.size() == 1)
		{
			exe_action = V_ACT[0];
			V_ACT.clear();
		}
		else if (agent_num == 3) {
			//cout << "No way to go！" << exe_flag_1 << exe_flag_2 << exe_flag_3 << " : " << index_1 << index_2 << index_3 << " : " << PI[0].size() << PI[1].size() << PI[2].size() << endl;
			exe_flag_1 = exe_flag_2 = exe_flag_3 = true;
		}
		else {
			exe_flag_1 = exe_flag_2 = exe_flag_3 = exe_flag_4 = exe_flag_5 = true;
		}
		if (node_rescue[exe_action] == false) {
			weighted_repair_cost += action_fine[exe_action] * g.getImportance(exe_action);
			node_rescue[exe_action] = true;
			time_state_1.insert(make_pair(action_fine[exe_action], exe_action));

			cout << exe_action << " Time = " << action_fine[exe_action] << endl;
		}
		cout << "rest_demand_nodes.size() = " << rest_demand_nodes.size() << endl;
		rest_demand_nodes.erase(exe_action);

		switch (action_agent[exe_action])
		{
		case 1:exe_flag_1 = true; break;
		case 2:exe_flag_2 = true; break;
		case 3:exe_flag_3 = true; break;
		case 4:exe_flag_4 = true; break;
		case 5:exe_flag_5 = true; break;
		default:break;
		}
		if (rest_demand_nodes.size() == 0) break;
	}
	// Calculate final objective function
	cout << "object_func_rescue = " << weighted_repair_cost << endl;
	p.fitness = weighted_repair_cost;
	return weighted_repair_cost;
}

void VNS_main(Graph& g, int agent_num, int TRAIN_TIME, int train_index, int MAX_GENERATION, int Imax, individual& p, double& best_fitness) {
	int M = demand_node_store.size();
	initialize();
	Initial();
	seed = (1.0 / (TRAIN_TIME + 1)) * (train_index + 1);
	warmup_random(seed);
	// Initialize population				
	//encode_rescue(g, old_pop_ptr, M);
	dpMethod(g, agent_num, p);
	//bincode(p);
	best_fitness = decode_and_function1(p, g, agent_num, true);

	cout << "Repair schedule：" << time_state.size() << endl;
	for (auto iter = time_state.begin(); iter != time_state.end(); ++iter)
	{
		cout << iter->first << " ";
	}
	cout << endl;
}

void VNS_main_rescue(Graph& g, int agent_num, int TRAIN_TIME, int train_index, int MAX_GENERATION, int Imax, individual& p, double& best_fitness) {

	initialize();
	seed = (1.0 / (TRAIN_TIME + 1)) * (train_index + 1);
	warmup_random(seed);
	// Initialize population				
	//bincode_rescue(p, agent_num);
	best_fitness = CalObj_rescue(p, g, agent_num, false);

	cout << "Transport schedule：" << time_state_1.size() << endl;
	for (auto iter = time_state_1.begin(); iter != time_state_1.end(); ++iter)
	{
		cout << iter->first << " ";
	}
	cout << endl;
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

int main() {
	vector<int> node_num_store = { 40 };
	vector<int> edge_num_store = { 60 };
	vector<double> demand_pro_store = { 0.6 };
	vector<double> damaged_pro_store = { 0.7 };
	map<int, int> AGENT_NUM_store =
	{
		{ 40, 3 },
		{ 60, 5 },
		{ 80, 5 },
	};

	int GRAPH_NUM = 10;

	int TRAIN_TIME = 1;

	int MAX_GENERATION;		    /* Number of iterations */

	int Imax = 20;

	string common_path = "E:\\Test_EXP\\90 Road Network Cases\\";//the path of the road network dataset

	string common_path2 = "E:\\Test_EXP\\BD\\";//Default storage path for experimental results：
	//when you test the algorithm, please use the path suggested on the Githup.
	//For example：copy the whole BD folder to your test path, such as "E:\Test_EXP\"


	input();

	for (int i_index = 0; i_index < node_num_store.size(); ++i_index) {
		cout << "node_index = " << i_index << endl;
		int node_num = node_num_store[i_index];
		int edge_num = edge_num_store[i_index];
		int AGENT_NUM = AGENT_NUM_store[node_num];
		switch (node_num) {
		case 40:
			MAX_GENERATION = 200;
			break;
		case 60:
			MAX_GENERATION = 300;
			break;
		case 80:
			MAX_GENERATION = 400;
			break;
		default: break;
		}
		char node[10], edge[10];
		sprintf_s(node, "%d", node_num);
		sprintf_s(edge, "%d", edge_num);
		string nodestr(node);
		string edgestr(edge);
		string node_edge = nodestr + "-" + edgestr + "\\";
		// Three different damage rates
		for (int j_index = 0; j_index < demand_pro_store.size(); ++j_index) {
			double demand_node_pro = demand_pro_store[j_index];
			double damaged_edge_pro = damaged_pro_store[j_index];
			char de_pro[10], da_pro[10];
			sprintf_s(de_pro, "%.1f", demand_node_pro);
			sprintf_s(da_pro, "%.1f", damaged_edge_pro);
			string deprostr(de_pro);
			string daprostr(da_pro);
			string demand_damage = deprostr + "-" + daprostr + "\\";
			// Total 10 sets of data
			for (int graph_index = 1; graph_index <= 10; ++graph_index) {
				string read_path = common_path + node_edge + demand_damage + to_string(graph_index) + "\\";
				cout << read_path << endl;
				Graph g(node_num);
				createGraph(g, read_path, AGENT_NUM);

				for (int train_index = 0; train_index < TRAIN_TIME; ++train_index) {
					sleep((clock_t)3 * CLOCKS_PER_SEC);
					printf_s("experiment %d\n", train_index + 1);

					individual indiv, best_indiv;
					double best_obj = INF;

					individual rescue, best_rescue;
					double best_obj_rescue = INF;
					// Record
					clock_t start, finish;
					double duration = 0;
					start = clock();
					VNS_main(g, AGENT_NUM, TRAIN_TIME, train_index, MAX_GENERATION, Imax, best_indiv, best_obj);
					best_indiv.bincode = indiv.bincode;
					if (!test(g)) continue;
					VNS_main_rescue(g, AGENT_NUM, TRAIN_TIME, train_index, MAX_GENERATION, Imax, best_rescue, best_obj_rescue);
					best_rescue.bincode = rescue.bincode;
					finish = clock();

					char str_node[20];
					sprintf_s(str_node, "agent_node1", graph_index, AGENT_NUM);
					string tempstring_node(str_node);
					string filename_node = tempstring_node + ".txt";

					ofstream outfile_node;
					outfile_node.open(read_path + filename_node, ofstream::app);

					for (auto node_iter = demand_node_store.begin(); node_iter != demand_node_store.end(); ++node_iter)
					{
						int agent = agent_node[*node_iter];
						outfile_node << *node_iter << " " << agent << endl;
						cout << *node_iter << " " << agent << endl;
					}
					outfile_node.close();/**/

					char str[20];
					sprintf_s(str, "g%d_BD", graph_index);
					string tempstring(str);
					string filename = tempstring + ".txt";

					string write_path = common_path2 + node_edge + demand_damage + to_string(AGENT_NUM) + "_agent\\" + "Data\\";
					cout << write_path << endl;
					ofstream outfile;
					outfile.open(write_path + filename, ofstream::app);

					outfile << "BD:" << endl;
					outfile << "node_num = " << node_num << "edge_num = " << edge_num << ";" << demand_node_pro << "_" << damaged_edge_pro << endl;
					outfile << "/***********************BD result***********************/" << endl;
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

					outfile << "encode sequence: ";
					for (int i = 0; i < best_indiv.bincode.size(); ++i) {
						outfile << best_indiv.bincode[i] << " ";
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
					cout << read_path << endl;
					cout << "best_obj =" << best_obj << endl;
					cout << "best_obj_RESCUE =" << best_obj_rescue << endl;
					outfile.close();

					char str2[20];
					sprintf_s(str2, "g%d_BD", graph_index);
					string tempstring2(str2);
					string filename2 = tempstring2 + "-object.txt";
					ofstream outfile2;
					outfile2.open(write_path + filename2, ofstream::app);
					outfile2 << best_obj_rescue << endl;
					outfile2.close();

					char str3[20];
					sprintf_s(str3, "g%d_BD", graph_index);
					string tempstring3(str3);
					string filename3 = tempstring3 + "-time.txt";
					ofstream outfile3;
					outfile3.open(write_path + filename3, ofstream::app);
					outfile3 << static_cast<double>(finish - start) / CLOCKS_PER_SEC << endl;
					outfile3.close();

					sleep(1000);
					cout << endl;
				}
				cout << "*******************************************************************" << endl;
			}
		}
	}
	cout << "successful!\n" << endl;

	system("pause");

	return 0;
}