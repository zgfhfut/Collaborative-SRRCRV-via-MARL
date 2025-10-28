#include <stack>
#include <thread>
#include <mutex>
#include <windows.h>
#include <sstream>
#include <iomanip> 
#include<ctime>
#include<iostream>
#include "road.h"
#include "random.h"
#include <iostream>
#include <random>
#include <thread>
/********************************CMARL******************************/

using namespace std;

mutex action_mutex;//Operation lock for action set

mutex road_state_mutex;//Operation lock for state space

mutex try_mutex;

map<int, bool> node_rescue;
map<double, set<Line>> all_time_state;
map<double, Line> time_state;
map<double, Line> times;
map<double, int> times1, times2, times3, times4, times5;
map<double, int> time_state_1;//Delivery time for all demand points
map<int, int> agent_node;//Node partitioning
/************************/
/*Enter basic chart information here*/
/************************************************************************/
set<int> reserve_node_store;//Store reserve points

set<int> node_store;//Store road network nodes

set<int> const_demand_node_store;//Store original road network demand points before transformation
set<int> const_demand_node_store1;//Store original road network demand points before transformation, used for initialization

set<int> demand_node_store;//Damaged demand points
//set<int> demand_node_store1;//Demand points for material distribution

set<Line> edge_store;//Store road network edges

set<Line> const_damaged_edge_store;//Store original road network damaged edges before transformation, used for initialization
set<Line> damaged_edge_store;

map<int, double> max_allowed_dis_store_0;//Store maximum allowed distance

map<int, double> max_allowed_dis_store_1;//Store maximum allowed distance

map<int, double> max_allowed_dis_store_2;//Store maximum allowed distance

map<int, double> max_allowed_dis_store_3;//Store maximum allowed distance

map<int, double> max_allowed_dis_store_4;//Store maximum allowed distance
/***********************************************************************/
/*
The map container is a key-value pair collection. To have one label correspond to one state, the state set is used as the key.
∵ The map requires unique keys, ∴ using the state set as the key ensures no duplicate states are inserted during storage.
This makes processing easiest, rather than the conventional way (using label as key).
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

set<Line> unexe_actions_set;//Record unexecuted action set
set<int> unexe_actions_set1;//Record unexecuted action set

map<string, double> qvalue_store;//map<"state_label-action",qvalue>
map<string, double> qvalue_store_1;//map<"state_label-action",qvalue>
map<string, double> qvalue_store_2;//map<"state_label-action",qvalue>
map<string, double> qvalue_store_3;//map<"state_label-action",qvalue>
map<string, double> qvalue_store_4;//map<"state_label-action",qvalue>
map<string, double> qvalue_store_5;//map<"state_label-action",qvalue>
map<string, double> qvalue_store_rescue;
map<string, double> qvalue_store_6;//map<"state_label-action",qvalue>
map<string, double> qvalue_store_7;//map<"state_label-action",qvalue>
map<string, double> qvalue_store_8;//map<"state_label-action",qvalue>
map<string, double> qvalue_store_9;//map<"state_label-action",qvalue>
map<string, double> qvalue_store_10;//map<"state_label-action",qvalue>

map<long, string> temp_s_a_store;
map<long, string> temp_s_a_store1;//Store <state, state_action> in current decision
map<long, string> temp_s_a_store2;
map<long, string> temp_s_a_store3;
map<long, string> temp_s_a_store4;//Store <state, state_action> in current decision
map<long, string> temp_s_a_store5;
map<long, string> temp_s_a_store_rescue;
map<long, string> temp_s_a_store6;
map<long, string> temp_s_a_store7;
map<long, string> temp_s_a_store8;//Store <state, state_action> in current decision
map<long, string> temp_s_a_store9;
map<long, string> temp_s_a_store10;

map<long, string> local_opt_store;
map<long, string> local_opt_store1;//Store optimal action for current learned s
map<long, string> local_opt_store2;
map<long, string> local_opt_store3;
map<long, string> local_opt_store4;//Store optimal action for current learned s
map<long, string> local_opt_store5;
map<long, string> local_opt_store_rescue;
map<long, string> local_opt_store6;
map<long, string> local_opt_store7;
map<long, string> local_opt_store8;
map<long, string> local_opt_store9;
map<long, string> local_opt_store10;

map<set<double>, long> all_state_store;//map<state_set,state_label>
map<set<double>, long> all_state_store1;//map<state_set,state_label>

void initialize() {
	//thread_local std::mt19937 gen(std::random_device{}());  // Thread-independent random number generator
	srand((unsigned)time(NULL)); //Initialize random function
}

void Initial_Graph()
{
	qvalue_store.clear();
	qvalue_store_rescue.clear();
	qvalue_store_1.clear();
	qvalue_store_2.clear();
	qvalue_store_3.clear();
	qvalue_store_4.clear();
	qvalue_store_5.clear();
	qvalue_store_6.clear();
	qvalue_store_7.clear();
	qvalue_store_8.clear();
	qvalue_store_9.clear();
	qvalue_store_10.clear();

	local_opt_store.clear();
	local_opt_store_rescue.clear();
	local_opt_store1.clear();
	local_opt_store2.clear();
	local_opt_store3.clear();
	local_opt_store4.clear();
	local_opt_store5.clear();
	local_opt_store6.clear();
	local_opt_store7.clear();
	local_opt_store8.clear();
	local_opt_store9.clear();
	local_opt_store10.clear();

	all_state_store.clear();
	all_state_store1.clear();

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

	agent_node.clear();

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
		g.setImportance(index, 0);//Set reserve point importance
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

		g.setRescueTime(node, rescue_time);//Time spent delivering supplies at demand point

		//const_demand_node_store1.insert(node);

		node_rescue.insert(make_pair(node, false));
	}
	infile5.close();

	for (auto iter = const_damaged_edge_store.begin(); iter != const_damaged_edge_store.end(); ++iter)
	{
		g.setConnectionStatus(*iter, false);
	}

	/**********************g.setIocation**********************/
	string agent_1 = read_path + "agent_node1.txt";

	ifstream infile7;
	infile7.open(agent_1);
	int j = 0;
	while (j < node_store.size())
	{
		int node, agent;
		string oneline;
		getline(infile7, oneline);
		stringstream ssin(oneline);

		ssin >> node;
		ssin >> agent;

		agent_node.insert(make_pair(node, agent));
		j++;
	}
	//cout << agent_node.size() << endl;
	infile7.close();
}


// Generate random integer between [a,b)
int getRandom(int lowerRound, int upperRound)
{
	int value;

	value = lowerRound + (int)(upperRound - lowerRound) * rand() / (RAND_MAX + 1);

	return value;
}

//Generate floating point between (0,1)
double getDoubleRandom()
{
	double double_rand;

	double_rand = rand() / double((double)RAND_MAX + 1);
	//double_rand = randomperca();//Use normal distribution function to get (0,1) random value, more random than rand()
	//Original way: double_rand = rand() / double(RAND_MAX + 1);
	return double_rand;
}

//String split function, can split string s according to given delimiter delim
//Use "-" as delimiter to separate "state_label" and "action" from "state_label-action"
void mySplitString(string s, string delim, vector<string>& ret)
{

	string::size_type split_pos = 0;//Record starting position of each split

	string::size_type index = s.find_first_of(delim, split_pos);
	/*
	s.find_first_of(arg1,arg2) finds the first occurrence of arg1 in s starting from index arg2, returns its position. Returns -1 if not found.
	*/

	while (index != -1)
	{
		ret.push_back(s.substr(split_pos, index - split_pos));//Store characters from split position to delimiter position into ret
		//|state_label|-action, store state_label into ret
		//s.substr(arg1,arg2) string截断function: start from arg1,截取arg2 characters

		split_pos = index + 1;//Record starting position of next split
		index = s.find_first_of(delim, split_pos);
	}

	//If there are elements after the last delimiter
	if (s.length() - split_pos > 0)
	{
		ret.push_back(s.substr(split_pos));
	}
}

void Initial_State2(Graph g, set<int>& demand_nodes_set, int agent_num)
{
	vector<double> dis_to;
	for (int i = 0; i < agent_num; i++) {
		g.shortestDis(i, dis_to);//Find shortest distance from all nodes to point 0, store in dis_to_0
		map<int, double> max_allowed_dis_store;
		switch (i)
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
		for (auto node_iter = demand_node_store.begin(); node_iter != demand_node_store.end(); ++node_iter)//Simplify action set
		{
			if (dis_to[*node_iter] <= max_allowed_dis_store[*node_iter])
			{
				demand_nodes_set.erase(*node_iter);
			}
		}
	}
	//cout << "demand_nodes_set.size() = " << demand_nodes_set.size() << endl;
	//cout << "state_set.size() = " << state_set.size() << endl;
}

long Initial_State1(Graph g, set<int>& actions_set, double dest)
{
	set<double> state_set;
	state_set.insert(dest);
	vector<double> dis_to;

	g.shortestDis(dest, dis_to);//Find shortest distance from all nodes to point 0, store in dis_to_0
	set<int> actions_node = actions_set;
	for (auto node_iter = actions_node.begin(); node_iter != actions_node.end(); ++node_iter)
	{
		if (dis_to[*node_iter] < INF)//Demand point reachable, store in action set 1
		{
			actions_set.erase(*node_iter);
			unexe_actions_set1.erase(*node_iter);
			//cout << *node_iter << " ";
		}
	}
	//cout <<"*****************actions_set.size() = " << actions_set.size() << endl;
	long state_label = dest;//state_label initialized to 0,表示start counting from 0
	long temp_state_label = all_state_store1.size();//Ensure next state update inserted into map can follow previous state
	pair<map<set<double>, long>::iterator, bool> insert_status;
	insert_status = all_state_store1.insert(make_pair(state_set, temp_state_label));

	if (insert_status.second)
		state_label = temp_state_label;	//Insert successful, return insertion position as state label

	else
		state_label = all_state_store1[state_set];//If insert failed, return original state label

	return state_label;
}

long Initial_State(Graph g, set<int>& demand_nodes_set, int last_index, set<Line>& actions_set)
{
	set<double> state_set;
	state_set.insert(last_index);
	actions_set.clear();
	vector<double> dis_to;

	g.shortestDis(last_index, dis_to);//Find shortest distance from all nodes to point 0, store in dis_to_0
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
	for (auto node_iter = demand_node_store.begin(); node_iter != demand_node_store.end(); ++node_iter)//Simplify action set
	{
		if (dis_to[*node_iter] < max_allowed_dis_store[*node_iter])
		{
			demand_nodes_set.erase(*node_iter);
		}
		else
		{
			//state_set.insert(*node_iter);//State table stores isolated demand points
		}
	}

	for (auto edge_iter = unexe_actions_set.begin(); edge_iter != unexe_actions_set.end(); ++edge_iter)
	{

		int action_value = (edge_iter->dest * 10000) + (edge_iter->src);//Action label
		//state_set.insert(action_value);//State table stores unprocessed damaged edges

		if (dis_to[edge_iter->src] != INF || dis_to[edge_iter->dest] != INF)//Demand point reachable, store in action set 1
		{
			actions_set.insert(*edge_iter);
		}

	}
	//cout << "unexe_actions_set.size() = " << unexe_actions_set.size() << endl;
	//cout << "state_set.size() = " << state_set.size() << endl;


	long state_label = 0;//state_label initialized to 0,表示start counting from 0
	long temp_state_label = all_state_store.size();//Ensure next state update inserted into map can follow previous state
	pair<map<set<double>, long>::iterator, bool> insert_status;
	insert_status = all_state_store.insert(make_pair(state_set, temp_state_label));

	if (insert_status.second)
		state_label = temp_state_label;	//Insert successful, return insertion position as state label

	else
		state_label = all_state_store[state_set];//If insert failed, return original state label
	//cout << "state_label = " << state_label << endl;
	return state_label;
}

double Find_Max_Q(map<string, double> poss_qvalue, vector<string>& max_state_action)
{
	double max_qvalue = 0;

	//Can only determine maximum value after scanning entire map container
	for (map<string, double>::iterator iter = poss_qvalue.begin(); iter != poss_qvalue.end(); ++iter)
	{
		if ((iter->second) >= max_qvalue)
		{
			max_qvalue = iter->second;
		}
	}

	/*
	∵ There may be multiple state_actions with same maximum Q value, store them in vector
	∴ To find all possible "state_actions", need to scan entire map again
	*/
	for (map<string, double>::iterator iter = poss_qvalue.begin(); iter != poss_qvalue.end(); ++iter)
	{
		if ((iter->second) == max_qvalue)
		{
			max_state_action.push_back(iter->first);
		}
	}
	//	cout <<"max_state_action = " << max_state_action.size() << endl;
	return max_qvalue;
}

//Search optimal action set space, update max_state_action
void UpdateLocalOPT(int agent_index, long state_label, map<string, double> poss_qvalue, vector<string>& max_state_action)
{
	map<long, string> temp_Local_opt;
	switch (agent_index)
	{
	case 0:temp_Local_opt = local_opt_store1; break;
	case 1:temp_Local_opt = local_opt_store2; break;
	case 2:temp_Local_opt = local_opt_store3; break;
	case 3:temp_Local_opt = local_opt_store4; break;
	case 4:temp_Local_opt = local_opt_store5; break;
	case 5:temp_Local_opt = local_opt_store6; break;
	case 6:temp_Local_opt = local_opt_store7; break;
	case 7:temp_Local_opt = local_opt_store8; break;
	case 8:temp_Local_opt = local_opt_store9; break;
	case 9:temp_Local_opt = local_opt_store10; break;
	default:
		break;
	}
	//local_opt_store stores current best repair strategy, check if state_label appears in it and is within possible action space. Add to max_state_action
	if ((temp_Local_opt.find(state_label) != temp_Local_opt.end()) && (poss_qvalue.find(temp_Local_opt[state_label]) != poss_qvalue.end()))
		max_state_action.push_back(temp_Local_opt[state_label]);
}

int ChooseAct1(double epsilon, vector<string>& poss_state_action, vector<string>& max_state_action)
{
	int action_node;

	string state_action_string;

	vector<string> temp_string_store;
	/*epsilon-greedy*/
	double rand_choose = getDoubleRandom();

	int count;
	//When random number less than epsilon, randomly select from optimal state-action set
	if (rand_choose < epsilon)
	{
		count = max_state_action.size();
		if (count > 1)
		{
			int index = getRandom(0, count);
			state_action_string = max_state_action[index];
		}
		else if (count == 1)
		{
			state_action_string = max_state_action[0];
		}
	}
	//When random number greater than epsilon, randomly select from possible state-action set
	else
	{
		//action_node = choose_node;
		count = poss_state_action.size();
		if (count > 1)
		{
			int index = getRandom(0, count);
			state_action_string = poss_state_action[index];
		}
		else if (count == 1)
		{
			state_action_string = poss_state_action[0];
		}
		/*double total_value = 0;
		//Calculate total value
		for (auto iter = poss_choose_action_dis.begin(); iter != poss_choose_action_dis.end(); ++iter)
			total_value += 1 / iter->second;

		//Roulette selection
		double fSlice = getDoubleRandom() * total_value;
		double fTotal = 0.0;
		for (auto iter2 = poss_choose_action_dis.begin(); iter2 != poss_choose_action_dis.end(); ++iter2) {
			fTotal += 1 / iter2->second;
			action_node = iter2->first;
			if (fTotal > fSlice)	break;
		}
		return action_node;*/

	}

	//Only proceed when state_action_string has value
	if (!state_action_string.empty())
	{
		mySplitString(state_action_string, "-", temp_string_store);//Split string with "-"

		action_node = stoi(temp_string_store[1]);//stoi（str_int）function converts specified string number str_int to corresponding int type

	}
	else
		action_node = 0;

	return action_node;
}

Line ChooseAct(double epsilon, vector<string>& poss_state_action, vector<string>& max_state_action)
{
	int action_int;
	string state_action_string;

	/*epsilon-greedy*/
	double rand_choose = getDoubleRandom();

	int count;
	//When random number less than epsilon, randomly select from optimal state-action set
	if (rand_choose < epsilon)
	{
		count = max_state_action.size();
		if (count > 1)
		{
			int index = getRandom(0, count);
			//cout << "indexmax = " << index << endl;
			state_action_string = max_state_action[index];
		}
		else if (count == 1)
		{
			state_action_string = max_state_action[0];
		}
	}
	//When random number greater than epsilon, randomly select from possible state-action set
	else
	{
		count = poss_state_action.size();
		if (count > 1)
		{
			int index = getRandom(0, count);
			//cout << "index = " << index << endl;
			state_action_string = poss_state_action[index];
		}
		else if (count == 1)
		{
			state_action_string = poss_state_action[0];
		}
	}

	/*Separate "action" from selected "state_action"*/
	vector<string> temp_string_store;

	//Only proceed when state_action_string has value
	if (!state_action_string.empty())
	{
		mySplitString(state_action_string, "-", temp_string_store);//Split string with "-"

		action_int = stoi(temp_string_store[1]);//stoi（str_int）function converts specified string number str_int to corresponding int type

	}
	else
		action_int = 0;

	/*Convert action_int back to src and dest*/
	int dest = action_int / 10000;
	int src = action_int - dest * 10000;
	Line choose_action_line(src, dest);

	return choose_action_line;
}

void Policy2(Graph g, Line& action_line, long state_label, int agent_index, int& last_dest, bool exe_flag,
	double epsilon, double total_cost, set<Line> actions_set, double& time, set<Line> V_ACT, bool& flag)
{
	initialize();
	int next_dest;
	int action_int = 0;
	string state_action;
	double Q_value;
	vector<double> dis_array;
	map<string, double> poss_qvalue;//Store all possible Q values in state s
	vector<string> poss_state_action;//Store all possible <state-action> pairs in state s
	vector<string> max_state_action;//Store <state-action> pairs corresponding to maximum Q value in state s

	//Filter reachable and unrepaired edges, get all corresponding Q(s,a) values for current state s
	//if (exe_flag) {
		//const lock_guard<mutex> lock(try_mutex); // Acquire exclusive lock
	for (auto edge_iter = actions_set.begin(); edge_iter != actions_set.end(); ++edge_iter)
	{
		if (V_ACT.find(*edge_iter) == V_ACT.end())
		{
			action_int = (edge_iter->dest) * 10000 + (edge_iter->src);
			state_action = to_string(state_label) + "-" + to_string(action_int);

			poss_state_action.push_back(state_action);

			switch (agent_index)
			{
			case 0:Q_value = qvalue_store_1[state_action]; break;
			case 1:Q_value = qvalue_store_2[state_action]; break;
			case 2:Q_value = qvalue_store_3[state_action]; break;
			case 3:Q_value = qvalue_store_4[state_action]; break;
			case 4:Q_value = qvalue_store_5[state_action]; break;
			default:break;
			}
			//Q_value = qvalue_store[state_action];
			poss_qvalue.insert(make_pair(state_action, Q_value));
		}


		if (poss_state_action.size() == 0) {
			//cout << "No path available! time = " <<time << endl;
			time = total_cost + 5;
			flag = false;
		}

		//Determine maximum Q value in state s and corresponding <state-action pair>
		double max_Q = Find_Max_Q(poss_qvalue, max_state_action);

		//Update optimal action set into max_state_action set
		UpdateLocalOPT(agent_index, state_label, poss_qvalue, max_state_action);

		//Greedy decision to select action
		action_line = ChooseAct(epsilon, poss_state_action, max_state_action);

	}

	//Determine time cost required after executing selected action
	unexe_actions_set.erase(action_line);

	g.shortestDis(last_dest, dis_array);

	next_dest = dis_array[action_line.dest] > dis_array[action_line.src] ? action_line.dest : action_line.src;

	double repair_cost = dis_array[action_line.dest] > dis_array[action_line.src] ? dis_array[action_line.src] : dis_array[action_line.dest];

	repair_cost += g.getRepairTime(action_line) + g.getWeight(action_line);

	time = total_cost + repair_cost;
	//cout << "time = " << time << "repair_cost = " << repair_cost << endl;
	last_dest = next_dest;//Update position

	flag = true;
	//}
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

//Bubble sort [descending order]
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

double ExeAct1(int action_node, Graph& g, int agent_num, map<int, vector<double>>& dis_aft)
{
	for (int i = 0; i < agent_num; ++i)
	{
		vector<double> dis_array;
		g.shortestDis(i, dis_array);
		dis_aft[i] = dis_array;
	}
	return 0;
}

double ExeAct(Line action_line, Graph& g, int agent_num,
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

	return edge_weight;//Return edge weight
}
//Calculate reward, repair team, transport team
double CalReward1(Graph g, int rescue_src, int rescue_dest, double dis)
{
	double reward = 0;
	double lambda = 0.1;
	bool re_flag = false;

	//cout << rescue_src << " " << rescue_dest << ":" << dis;
	reward = g.getImportance(rescue_dest) * 10 / dis;
	//reward += get_value(rescue_src, rescue_dest, alpha2, beta, dis);

	//cout <<"reward = " << g.getImportance(rescue_dest)<<" "<< dis << " " << reward << endl;
	return reward;
}

double CalReward(Graph g, int agent_num, double last_total_cost, double total_cost,
	map<int, vector<double>> dis_pre, map<int, vector<double>> dis_aft)
{
	double reward = 0;
	double lambda = 0.1;
	bool re_flag = false;

	for (auto node_iter = node_store.begin(); node_iter != node_store.end(); ++node_iter)
	{
		if (dis_aft[0][*node_iter] <= max_allowed_dis_store_0[*node_iter]
			&& dis_pre[0][*node_iter] > max_allowed_dis_store_0[*node_iter])
		{
			reward += (lambda * g.getImportance(*node_iter) * 100 / dis_aft[0][*node_iter]) + ((1 - lambda) * g.getImportance(*node_iter) * 100 / total_cost);
			re_flag = true;
		}
		if (dis_aft[1][*node_iter] <= max_allowed_dis_store_1[*node_iter]
			&& dis_pre[1][*node_iter] > max_allowed_dis_store_1[*node_iter])
		{
			reward += (lambda * g.getImportance(*node_iter) * 100 / dis_aft[1][*node_iter]) + ((1 - lambda) * g.getImportance(*node_iter) * 100 / total_cost);
			re_flag = true;
		}
		if (dis_aft[2][*node_iter] <= max_allowed_dis_store_2[*node_iter]
			&& dis_pre[2][*node_iter] > max_allowed_dis_store_2[*node_iter])
		{
			reward += (lambda * g.getImportance(*node_iter) * 100 / dis_aft[2][*node_iter]) + ((1 - lambda) * g.getImportance(*node_iter) * 100 / total_cost);
			re_flag = true;
		}
		if (agent_num == 5 && dis_aft[3][*node_iter] <= max_allowed_dis_store_3[*node_iter]
			&& dis_pre[3][*node_iter] > max_allowed_dis_store_3[*node_iter])
		{
			reward += (lambda * g.getImportance(*node_iter) * 100 / dis_aft[3][*node_iter]) + ((1 - lambda) * g.getImportance(*node_iter) * 100 / total_cost);
			re_flag = true;
		}
		if (agent_num == 5 && dis_aft[4][*node_iter] <= max_allowed_dis_store_4[*node_iter]
			&& dis_pre[4][*node_iter] > max_allowed_dis_store_4[*node_iter])
		{
			reward += (lambda * g.getImportance(*node_iter) * 100 / dis_aft[4][*node_iter]) + ((1 - lambda) * g.getImportance(*node_iter) * 100 / total_cost);
			re_flag = true;
		}/**/
	}

	if (!re_flag)
	{
		if (last_total_cost != 0)
			reward += 1 / (total_cost - last_total_cost);
		else
			reward += 1 / total_cost;
	}
	//cout << "reward = "<< reward << endl;
	return reward;
}

//Evaluate road repair status under independent repair team = set of isolated demand points, repaired damaged road sections. agent_unexe is path already repaired by repair team
long EvalState1(Graph g, int agent_index, set<int> actions_set,
	int action_node, double total_cost)
{
	set<double> state_set;
	state_set.insert(action_node * 100);
	/*for (auto iter = time_state.begin(); iter != time_state.end(); iter++) {
		if (total_cost >= iter->first) {
			int action_value = (iter->second.dest) * 10000 + (iter->second.src);
			state_set.insert(action_value);
		}
		else {
			break;
		}
	}*/

	for (auto check_iter2 = actions_set.begin(); check_iter2 != actions_set.end(); ++check_iter2)
	{
		state_set.insert(*check_iter2);
	}
	//cout << "state_set.size() = " << state_set.size() << endl;
	//cout<<"total_cost = " << total_cost <<"unexe_actions_set1 = " << unexe_actions_set1.size()<<endl;
	const lock_guard<mutex> lock(road_state_mutex);//Lock state all_state_store
	long state_label;
	long temp_state_label = all_state_store1.size();
	//cout << "state_label = " << temp_state_label << endl;
	//cout << "all_state_store1.size() = " << all_state_store1.size() << endl;
	pair<map<set<double>, long>::iterator, bool> insert_status = all_state_store1.insert(make_pair(state_set, temp_state_label));

	if (insert_status.second)
		state_label = temp_state_label;
	else {
		state_label = all_state_store1[state_set];

	}

	return state_label;
}
//dis_aft[0][*check_iter2] <= max_allowed_dis_store_0[*check_iter2] || dis_aft[1][*check_iter2] <= max_allowed_dis_store_1[*check_iter2] || dis_aft[2][*check_iter2] <= max_allowed_dis_store_2[*check_iter2] ||dis_aft[3][*check_iter2] <= max_allowed_dis_store_3[*check_iter2] || dis_aft[4][*check_iter2] <= max_allowed_dis_store_4[*check_iter2]
long EvalState(Graph g, int agent, Line action_line, set<int>& demand_nodes_set, set<Line>& actions_set,
	double total_cost, int last_dest, map<int, vector<double>> dis_aft, double& weighted_repair_cost)
{
	set<double> state_set;
	state_set.insert(last_dest);
	//state_set.insert(action_line.dest * 10000 + action_line.src);
	//Filter reachable damaged edges in next state from agent_unexe + add agent's unexecuted paths to state
	actions_set.clear();
	vector<double > dis_aft1;
	g.shortestDis(last_dest, dis_aft1);
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
	const lock_guard<mutex> lock(action_mutex);//Lock action set, lock_guard automatically unlocks when scope ends
	set<int> demand_nodes_set_temp = demand_nodes_set;
	for (auto check_iter2 = demand_nodes_set_temp.begin(); check_iter2 != demand_nodes_set_temp.end(); ++check_iter2)
	{
		if (dis_aft[agent][*check_iter2] < INF)
		{
			demand_nodes_set.erase(*check_iter2);
			unexe_actions_set1.erase(*check_iter2);
			weighted_repair_cost += g.getImportance(*check_iter2) * total_cost;
		}
	}

	for (auto check_iter = unexe_actions_set.begin(); check_iter != unexe_actions_set.end(); ++check_iter)
	{
		int action_value = (check_iter->dest) * 10000 + (check_iter->src);
		//state_set.insert(action_value);

		if (dis_aft1[check_iter->src] != INF || dis_aft1[check_iter->dest] != INF)
		{
			actions_set.insert(*check_iter);
		}
	}/**/
	//cout << "demand_nodes_set = " << demand_nodes_set.size() << endl;
	long state_label;
	long temp_state_label = all_state_store.size();

	pair<map<set<double>, long>::iterator, bool> insert_status = all_state_store.insert(make_pair(state_set, temp_state_label));
	if (insert_status.second)
		state_label = temp_state_label;
	else
		state_label = all_state_store[state_set];
	//cout << "state_label = " << state_label << endl;
	return state_label;
}

//Update Q table, transport team, repair team. Each time execute action_node in state state_label
void UpdateQValue1(int agent_index, long state_label, double alpha, double gamma, double reward, double max_Q, int action_node)
{
	double Q_value = 0;

	string state_action = to_string(state_label) + "-" + to_string(action_node);

	switch (agent_index)
	{
	case 0:
		Q_value = qvalue_store_6[state_action];
		qvalue_store_6[state_action] = (1 - alpha) * Q_value + alpha * (reward + gamma * max_Q);
		break;
	case 1:
		Q_value = qvalue_store_7[state_action];
		qvalue_store_7[state_action] = (1 - alpha) * Q_value + alpha * (reward + gamma * max_Q);
		break;
	case 2:
		Q_value = qvalue_store_8[state_action];
		qvalue_store_8[state_action] = (1 - alpha) * Q_value + alpha * (reward + gamma * max_Q);
		break;
	case 3:
		Q_value = qvalue_store_9[state_action];
		qvalue_store_9[state_action] = (1 - alpha) * Q_value + alpha * (reward + gamma * max_Q);
		break;
	case 4:
		Q_value = qvalue_store_10[state_action];
		qvalue_store_10[state_action] = (1 - alpha) * Q_value + alpha * (reward + gamma * max_Q);
		break;
	default:break;
	}
	//Q_value = qvalue_store_rescue[state_action];
	//qvalue_store_rescue[state_action] = (1 - alpha) * Q_value + alpha * (reward + gamma * max_Q);
	//cout << "max_Q =" << max_Q << "Q_value = " << Q_value << endl;
}

void UpdateQValue(int agent_index, long state_label, double alpha, double gamma, double reward, double max_Q, Line action_line)
{
	double Q_value = 0;

	int action_int = action_line.dest * 10000 + action_line.src;
	string state_action = to_string(state_label) + "-" + to_string(action_int);

	switch (agent_index)
	{
	case 0:
		Q_value = qvalue_store_1[state_action];
		qvalue_store_1[state_action] = (1 - alpha) * Q_value + alpha * (reward + gamma * max_Q);
		break;
	case 1:
		Q_value = qvalue_store_2[state_action];
		qvalue_store_2[state_action] = (1 - alpha) * Q_value + alpha * (reward + gamma * max_Q);
		break;
	case 2:
		Q_value = qvalue_store_3[state_action];
		qvalue_store_3[state_action] = (1 - alpha) * Q_value + alpha * (reward + gamma * max_Q);
		break;
	case 3:
		Q_value = qvalue_store_4[state_action];
		qvalue_store_4[state_action] = (1 - alpha) * Q_value + alpha * (reward + gamma * max_Q);
		break;
	case 4:
		Q_value = qvalue_store_5[state_action];
		qvalue_store_5[state_action] = (1 - alpha) * Q_value + alpha * (reward + gamma * max_Q);
		break;
	default:break;
	}
	//Q_value = qvalue_store[state_action];
	//qvalue_store[state_action] = (1 - alpha) * Q_value + alpha * (reward + gamma * max_Q);
}

void Q_learning1(long state_label, long next_state_label, int agent_index, double alpha, double gamma, double reward, int action_node, const set<int> actions_set)
{
	double max_Q = 0;
	string state_action;

	map<string, double> poss_qvalue;
	vector<string> max_state_action;

	//Filter reachable and unsatisfied points, get all corresponding Q(s,a) values for next state s'
	for (auto edge_iter = actions_set.begin(); edge_iter != actions_set.end(); ++edge_iter)
	{
		double Q_value;

		state_action = to_string(next_state_label) + "-" + to_string(*edge_iter);//State-action label

		switch (agent_index)
		{
		case 0:Q_value = qvalue_store_6[state_action]; break;
		case 1:Q_value = qvalue_store_7[state_action]; break;
		case 2:Q_value = qvalue_store_8[state_action]; break;
		case 3:Q_value = qvalue_store_9[state_action]; break;
		case 4:Q_value = qvalue_store_10[state_action]; break;
		default:break;
		}
		//Q_value = qvalue_store_rescue[state_action];
		poss_qvalue.insert(make_pair(state_action, Q_value));
	}
	max_Q = Find_Max_Q(poss_qvalue, max_state_action);//Find maximum Q value

	UpdateQValue1(agent_index, state_label, alpha, gamma, reward, max_Q, action_node);//Update Q table after executing one action
}

void Q_learning(long state_label, long next_state_label, int agent_index, double alpha, double gamma, double reward, Line action_line, const set<Line> actions_set)
{
	int action_int = 0;
	double max_Q = 0;
	string state_action;

	map<string, double> poss_qvalue;
	vector<string> max_state_action;

	//Filter reachable and unrepaired edges, get all corresponding Q(s,a) values for next state s'
	for (auto edge_iter = actions_set.begin(); edge_iter != actions_set.end(); ++edge_iter)
	{
		double Q_value;
		action_int = (edge_iter->dest) * 10000 + (edge_iter->src);
		state_action = to_string(next_state_label) + "-" + to_string(action_int);//State-action label
		switch (agent_index)
		{
		case 0:Q_value = qvalue_store_1[state_action]; break;
		case 1:Q_value = qvalue_store_2[state_action]; break;
		case 2:Q_value = qvalue_store_3[state_action]; break;
		case 3:Q_value = qvalue_store_4[state_action]; break;
		case 4:Q_value = qvalue_store_5[state_action]; break;
		default:break;
		}
		//Q_value = qvalue_store[state_action];
		poss_qvalue.insert(make_pair(state_action, Q_value));
	}

	max_Q = Find_Max_Q(poss_qvalue, max_state_action);//Find maximum Q value

	UpdateQValue(agent_index, state_label, alpha, gamma, reward, max_Q, action_line);//Update Q table after executing one action
}

bool Policy1(Graph g, set<int> actions, int& action_node, long state_label, int agent_index, int& last_dest,
	double epsilon, double total_cost, set<int>& actions_set, double& time, double& repair_cost, double& object_func_rescue, set<int> V_ACT_r)
{
	//cout << "object_func_rescue=========================== = " << object_func_rescue << endl;
	string state_action;
	double Q_value;
	actions_set.clear();
	map<string, double> poss_qvalue;//Store all possible Q values in state s
	vector<string> poss_state_action;//Store all possible <state-action> pairs in state s
	vector<string> max_state_action;//Store <state-action> pairs corresponding to maximum Q value in state s
	vector<double> dis_array;
	vector<Line> s;
	const lock_guard<mutex> lock(action_mutex);//Lock action set, lock_guard automatically unlocks when scope ends
	for (auto iter = time_state.begin(); iter != time_state.end(); iter++) {
		//cout<<"iter->second = "<< time_state.size()<<endl;
		if (time >= iter->first) {
			g.setConnectionStatus(iter->second, true);
			s.push_back(iter->second);
		}
		else break;
	}
	g.shortestDis(last_dest, dis_array);
	//cout << "iter->second = " << s.size() << endl;
	for (auto iter = s.begin(); iter != s.end(); iter++) g.setConnectionStatus(*iter, false);

	for (auto check_iter2 = actions.begin(); check_iter2 != actions.end(); ++check_iter2)
	{
		if (dis_array[*check_iter2] != INF && V_ACT_r.find(*check_iter2) == V_ACT_r.end())
		{
			actions_set.insert(*check_iter2);//Reachable demand point
			state_action = to_string(state_label) + "-" + to_string(*check_iter2);//Current state - corresponding possible action label

			poss_state_action.push_back(state_action);//Store possible action

			switch (agent_index)
			{
			case 0:Q_value = qvalue_store_6[state_action]; break;
			case 1:Q_value = qvalue_store_7[state_action]; break;
			case 2:Q_value = qvalue_store_8[state_action]; break;
			case 3:Q_value = qvalue_store_9[state_action]; break;
			case 4:Q_value = qvalue_store_10[state_action]; break;
			default:break;
			}
			//Q_value = qvalue_store_rescue[state_action];
			//Filter reachable and unrepaired edges, get all corresponding Q(s,a) values for current state s
			poss_qvalue.insert(make_pair(state_action, Q_value));//Find Q_value under corresponding label state_action from corresponding repair team's Q table
		}
	}

	if (poss_state_action.size() != 0) {
		Find_Max_Q(poss_qvalue, max_state_action);

		//Update optimal action set into max_state_action set
		UpdateLocalOPT(agent_index + 5, state_label, poss_qvalue, max_state_action);

		//Greedy decision to select action
		action_node = ChooseAct1(epsilon, poss_state_action, max_state_action);
	}

	//Determine maximum Q value in state s and corresponding <state-action pair>
	else {
		time = total_cost + 5;
		//cout << "Time extended!" << total_cost << endl;
		return false;
	}
	//cout << "Selected action:" <<  action_node << endl;
	//Determine time cost required after executing selected action
	repair_cost = dis_array[action_node];

	double tmp_cost = total_cost + repair_cost;

	if (!node_rescue[action_node]) tmp_cost += g.getRescueTime(action_node);//Unsatisfied demand point, add delivery time

	last_dest = action_node;//Update current position
	time = tmp_cost;
	object_func_rescue = object_func_rescue + time * g.getImportance(action_node);
	//cout << "action_node = " << action_node << "tmp_cost = " << tmp_cost << "object_func_rescue " << object_func_rescue << endl;
	return true;
}

bool Policy(Graph g, Line& action_line, long state_label, int agent_index, int& last_dest,
	double epsilon, double total_cost, double& time, set<Line> V_ACT)
{
	const lock_guard<mutex> lock(action_mutex);//Lock action set, lock_guard automatically unlocks when scope ends
	int next_dest = last_dest;
	int action_int = 0;
	string state_action;
	double Q_value;
	vector<double> dis_array;
	map<string, double> poss_qvalue;//Store all possible Q values in state s
	vector<string> poss_state_action;//Store all possible <state-action> pairs in state s
	vector<string> max_state_action;//Store <state-action> pairs corresponding to maximum Q value in state s
	bool flag = false;
	vector<double> temp_travel_dis;
	set<Line> actions_set;
	g.shortestDis(next_dest, temp_travel_dis);
	for (set<Line>::iterator iter = unexe_actions_set.begin(); iter != unexe_actions_set.end(); ++iter) {
		if ((temp_travel_dis[iter->src] < INF || temp_travel_dis[iter->dest] < INF) && V_ACT.find(*iter) == V_ACT.end()) {
			actions_set.insert(*iter);
		}
	}
	for (auto edge_iter = actions_set.begin(); edge_iter != actions_set.end(); ++edge_iter)
	{
		if (V_ACT.find(*edge_iter) == V_ACT.end()) {

			action_int = (edge_iter->dest) * 10000 + (edge_iter->src);
			state_action = to_string(state_label) + "-" + to_string(action_int);

			poss_state_action.push_back(state_action);

			switch (agent_index)
			{
			case 0:Q_value = qvalue_store_1[state_action]; break;
			case 1:Q_value = qvalue_store_2[state_action]; break;
			case 2:Q_value = qvalue_store_3[state_action]; break;
			case 3:Q_value = qvalue_store_4[state_action]; break;
			case 4:Q_value = qvalue_store_5[state_action]; break;
			default:break;
			}
			//Q_value = qvalue_store[state_action];
			poss_qvalue.insert(make_pair(state_action, Q_value));
		}
	}
	if (!flag) {
		if (poss_state_action.size() == 0) {
			cout << "No path available! time = " << time << endl;
			time = total_cost + 5;
			return false;
		}

		//Determine maximum Q value in state s and corresponding <state-action pair>
		double max_Q = Find_Max_Q(poss_qvalue, max_state_action);

		//Update optimal action set into max_state_action set
		UpdateLocalOPT(agent_index, state_label, poss_qvalue, max_state_action);

		//Greedy decision to select action
		action_line = ChooseAct(epsilon, poss_state_action, max_state_action);

	}
	//Determine time cost required after executing selected action
	actions_set.erase(action_line);

	g.shortestDis(last_dest, dis_array);

	next_dest = dis_array[action_line.dest] > dis_array[action_line.src] ? action_line.dest : action_line.src;

	double repair_cost = dis_array[action_line.dest] > dis_array[action_line.src] ? dis_array[action_line.src] : dis_array[action_line.dest];
	//cout << "time = " << time << "repair_cost = " << repair_cost << endl;
	repair_cost += g.getRepairTime(action_line) + g.getWeight(action_line);

	last_dest = next_dest;//Update position

	g.setConnectionStatus(action_line, true);

	time = total_cost + repair_cost;

	if (unexe_actions_set.find(action_line) != unexe_actions_set.end()) {
		time_state.insert(make_pair(time, action_line));
	}
	unexe_actions_set.erase(action_line);

	return true;
}

//Repair teams work independently
double Pthread_learning(Graph g, int agent, int agent_num, int& last_dest, double epsilon, double alpha, double gamma, double lambda,
	vector<Line>& pi_1, set<int>& unexe_actions_set1, set<int> demand_nodes_1, double& object_func)
{
	initialize();
	set<Line> action;
	set<int> rest_demand_nodes = demand_nodes_1;
	double weighted_repair_cost = 0;
	double weighted_dis_cost = 0;
	double time_1 = 0;
	double total_cost_1 = 0;
	Line act_1(0, 0);
	long state_label = agent;

	vector<Line> V_ACT;//Buffer storing actions to be executed
	map<Line, int> action_agent;//Record action corresponding repair team
	map<Line, double> action_fine;//Record corresponding action execution time
	Line exe_action(0, 0);
	map<int, vector<double>> dis_pre;
	map<int, vector<double>> dis_aft;

	set<Line> v;

	vector<double> dis_to_0;
	vector<double> dis_to_1;
	vector<double> dis_to_2;
	vector<double> dis_to_3;
	vector<double> dis_to_4;

	while (!rest_demand_nodes.empty())
	{
		dis_pre.clear();
		dis_aft.clear();

		bool flag = true;
		flag = Policy(g, act_1, state_label, agent, last_dest, epsilon, total_cost_1, time_1, v);
		v.insert(act_1);
		action_agent[act_1] = agent;//Set repair team executing action act_1

		action_fine[act_1] = time_1;//Store time after executing act_1
		//cout << "action_fine[act_1] = " << action_fine[act_1] << endl;

		double last_total_cost = total_cost_1;
		total_cost_1 = time_1;
		ExeAct(act_1, g, agent_num, dis_pre, dis_aft);//Execute action		
		if (flag) {
			pi_1.push_back(act_1);
			int action_int = (act_1.dest) * 10000 + (act_1.src);
			string state_action = to_string(state_label) + "-" + to_string(action_int);
			long next_state_label = EvalState(g, agent, act_1, rest_demand_nodes, action, total_cost_1, last_dest, dis_aft, weighted_repair_cost);
			switch (agent)
			{
			case 0: {
				temp_s_a_store1.insert(make_pair(state_label, state_action));
				break;
			}
			case 1: {
				temp_s_a_store2.insert(make_pair(state_label, state_action));
				break;
			}
			case 2: {
				temp_s_a_store3.insert(make_pair(state_label, state_action));
				break;
			}
			case 3: {
				temp_s_a_store4.insert(make_pair(state_label, state_action));
				break;
			}
			case 4: {
				temp_s_a_store5.insert(make_pair(state_label, state_action));
				break;
			}
			default:break;
			}
			temp_s_a_store.insert(make_pair(state_label, state_action));
			double reward = CalReward(g, agent_num, last_total_cost, total_cost_1, dis_pre, dis_aft);
			//cout <<"agent =  "<< agent << "act_1 = " << act_1.src << " - " << act_1.dest << endl;
			Q_learning(state_label, next_state_label, agent, alpha, gamma, reward, act_1, action);

			state_label = next_state_label;

			/*dis_to_0.clear();
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
					weighted_repair_cost += g.getImportance(*node_iter) * action_fine[act_1];
					rest_demand_nodes.erase(*node_iter);
					unexe_actions_set1.erase(*node_iter);
					//cout << " Opened disaster point：" << *node_iter << " - " << dis_to_0[*node_iter] <<" - " << dis_to_1[*node_iter] << " - " << dis_to_2[*node_iter] << " - " << dis_to_3[*node_iter] <<" - " << dis_to_4[*node_iter] << endl;
				}
			}*/
			if (rest_demand_nodes.empty()) {
				//cout << "Repair completed!" << endl;
				break;
			}
			//cout << "rest_demand_nodes.size = " << rest_demand_nodes.size() << "unexe_actions_set1.size = " << unexe_actions_set1.size() << endl;
		}
	}
	g.shortestDis(0, dis_to_0);
	g.shortestDis(1, dis_to_1);
	g.shortestDis(2, dis_to_2);
	g.shortestDis(3, dis_to_3);
	g.shortestDis(4, dis_to_4);
	/*Repair completed*/
	for (auto node_iter = demand_nodes_1.begin(); node_iter != demand_nodes_1.end(); ++node_iter)
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

	//Calculate final objective function
	object_func = weighted_dis_cost * lambda + weighted_repair_cost * (1 - lambda);
	/*cout << " Final：" << time_state.size() << endl;
	for (auto iter = time_state.begin(); iter != time_state.end(); ++iter)
	{
		cout << iter->first << ":" << iter->second.src << "-" << iter->second.dest << " action_agent = " << action_agent[iter->second] << endl;
	}
	cout << endl;
	cout << "weighted_repair_cost = " << weighted_repair_cost << endl;*/
	//cout << "object = " << object_func << endl;
	return 0;
}

double Pthread_learning1(Graph g, int agent_index, int agent_num, int& last_dest,
	double epsilon, double alpha, double gamma, double lambda, vector<int>& pi_1, double& total_cost,
	set<int>& unexe_actions_set1, double& object_func_rescue, map<double, int>& time_state_r)//demand_nodes_set demand points, actions_set damaged points, agent_unexe reachable damaged
{
	set<int> demand_nodes_1, rest_demand_nodes;
	for (auto iter = unexe_actions_set1.begin(); iter != unexe_actions_set1.end(); ++iter)
	{
		if (agent_node[*iter] == agent_index + 1) {
			demand_nodes_1.insert(*iter);
		}
	}
	//cout << "agent = " << agent_index << "demand_nodes_1.size() = " << demand_nodes_1.size() << endl;
	rest_demand_nodes = demand_nodes_1;
	bool quit_flag = false;
	set<int> actions_set_6;
	double time_6 = 0;
	long state_label_6 = agent_index;
	double total_cost_6 = 0;
	int act_6 = agent_index;
	double repair_cost_1 = 0;

	int i = 1;
	vector<int> V_ACT_r;
	map<int, int> action_agent_r;//Record action corresponding repair team
	map<int, double> action_fine_r;//Record corresponding action execution time
	set<int> v_r;
	while (!rest_demand_nodes.empty()) {

		bool quit_flag = false;

		quit_flag = Policy1(g, rest_demand_nodes, act_6, state_label_6, agent_index, last_dest,
			epsilon, total_cost_6, actions_set_6, time_6, repair_cost_1, object_func_rescue, v_r);//Select execution action from action_sets
		v_r.insert(act_6);
		action_agent_r[act_6] = 1;//Set repair team executing action act_1

		action_fine_r[act_6] = time_6;//Store time after executing act_1

		V_ACT_r.push_back(act_6);//V_ACT.size()=333.......321

		//cout << "agent = " << agent_index << " act = " << act_6 << endl;

		if (node_rescue[act_6] == false) time_state_r.insert(make_pair(action_fine_r[act_6], i++));

		// object_func_rescue += action_fine_r[exe_act] * g.getImportance(exe_act);

		node_rescue[act_6] = true;

		//cout << "unexe_actions_set1 = " << unexe_actions_set1.size() << " exe_act  = " << exe_act <<"action_agent_r[exe_act] = "<< action_agent_r[exe_act] << endl;
		unexe_actions_set1.erase(act_6);
		rest_demand_nodes.erase(act_6);
		total_cost_6 = time_6;

		string state_action = to_string(state_label_6) + "-" + to_string(act_6);

		switch (agent_index)
		{
		case 0: {
			temp_s_a_store6.insert(make_pair(state_label_6, state_action));
			break;
		}
		case 1: {
			temp_s_a_store7.insert(make_pair(state_label_6, state_action));
			break;
		}
		case 2: {
			temp_s_a_store8.insert(make_pair(state_label_6, state_action));
			break;
		}
		case 3: {
			temp_s_a_store9.insert(make_pair(state_label_6, state_action));
			break;
		}
		case 4: {
			temp_s_a_store10.insert(make_pair(state_label_6, state_action));
			break;
		}
		default:break;
		}

		long next_state_label = EvalState1(g, agent_index, actions_set_6, act_6, time_6);//Next state label, update action set actions_set
		//cout << "state_label = " << next_state_label << endl;
		if (quit_flag) {
			pi_1.push_back(act_6);//Store this action

			double reward = CalReward1(g, last_dest, act_6, time_6);//Reward for executing action

			Q_learning1(state_label_6, next_state_label, agent_index, alpha, gamma, reward, act_6, actions_set_6);//Q learning process

			state_label_6 = next_state_label;
		}
		if (rest_demand_nodes.empty()) {
			break;
		}
	}
	//cout << "temp_func_rescue = " << object_func_rescue << endl;
	return 0;
}

double CalObj1(Graph g, int agent_num, double lambda)
{
	double weighted_repair_cost = 0;
	double object_func = 0;

	for (auto node = const_demand_node_store1.begin(); node != const_demand_node_store1.end(); node++) {
		weighted_repair_cost += time_state_1[*node] * g.getImportance(*node);
	}
	//Calculate final objective function
	object_func = weighted_repair_cost;

	return object_func;
}

double multi_learning(Graph g, int agent_num, double f_clock, double epsilon, double alpha, double gamma, double lambda, double& best_obj, bool& best)
{
	best = false;
	time_state.clear();
	temp_s_a_store.clear();
	temp_s_a_store1.clear();
	temp_s_a_store2.clear();
	temp_s_a_store3.clear();
	temp_s_a_store4.clear();
	temp_s_a_store5.clear();

	set<int> demand_nodes_set;
	demand_nodes_set.clear();
	demand_nodes_set = demand_node_store;
	map<double, int> time_state_r;

	vector<vector<string>> s_a_store;
	unexe_actions_set.clear();
	unexe_actions_set1.clear();
	unexe_actions_set = damaged_edge_store;//Store damaged edges
	unexe_actions_set1 = const_demand_node_store;
	map<double, Line> temp_state;
	vector<string> s_a_1, s_a_2, s_a_3, s_a_4, s_a_5;

	vector<Line> PI_1, PI_2, PI_3, PI_4, PI_5;

	vector<Line> T_pi_1, T_pi_2, T_pi_3, T_pi_4, T_pi_5;
	int last_dest_1 = 0, last_dest_2 = 1, last_dest_3 = 2, last_dest_4 = 3, last_dest_5 = 4;
	//cout << "demand_nodes_set = " << demand_nodes_set.size() << endl;
	//cout << " unexe_actions_set = " << unexe_actions_set.size() << endl;

	double object_func = 0, object_func1 = 0, object_func2 = 0, object_func3 = 0, object_func4 = 0, object_func5 = 0;
	set<int> demand_nodes_1, demand_nodes_2, demand_nodes_3, demand_nodes_4, demand_nodes_5;
	//Initial_State2(g, demand_nodes_set, 5);
	for (auto iter = unexe_actions_set1.begin(); iter != unexe_actions_set1.end(); ++iter)
	{
		switch (agent_node[*iter])
		{
		case 1: {
			demand_nodes_1.insert(*iter);
			break;
		}
		case 2: {
			demand_nodes_2.insert(*iter);
			break;
		}
		case 3: {
			demand_nodes_3.insert(*iter);
			break;
		}
		case 4: {
			demand_nodes_4.insert(*iter);
			break;
		}
		case 5: {
			demand_nodes_5.insert(*iter);
			break;
		}
		default:break;
		}
	}
	Initial_State1(g, demand_nodes_1, 0);
	Initial_State1(g, demand_nodes_2, 1);
	Initial_State1(g, demand_nodes_3, 2);
	Initial_State1(g, demand_nodes_4, 3);
	Initial_State1(g, demand_nodes_5, 4);
	/*Pthread_learning(g, 0, agent_num, last_dest_1, epsilon, alpha, gamma, lambda, T_pi_1, unexe_actions_set1, object_func1);
	Pthread_learning(g, 1, agent_num, last_dest_2, epsilon, alpha, gamma, lambda, T_pi_2, unexe_actions_set1, object_func2);
	Pthread_learning(g, 2, agent_num, last_dest_3, epsilon, alpha, gamma, lambda, T_pi_3, unexe_actions_set1, object_func3);
	Pthread_learning(g, 3, agent_num, last_dest_4, epsilon, alpha, gamma, lambda, T_pi_4, unexe_actions_set1, object_func4);
	Pthread_learning(g, 4, agent_num, last_dest_5, epsilon, alpha, gamma, lambda, T_pi_5, unexe_actions_set1, object_func5);*/

	//cout << "demand_nodes_1.size() = " << demand_nodes_1.size() << " " << demand_nodes_2.size() << " " << demand_nodes_3.size() << " " << demand_nodes_4.size() << " " << demand_nodes_5.size() << " " << endl;

	while (!unexe_actions_set1.empty()) {
		thread t1(Pthread_learning, g, 0, agent_num, ref(last_dest_1), epsilon, alpha, gamma, lambda, ref(T_pi_1), ref(unexe_actions_set1), demand_nodes_1, ref(object_func1));

		thread t2(Pthread_learning, g, 1, agent_num, ref(last_dest_2), epsilon, alpha, gamma, lambda, ref(T_pi_2), ref(unexe_actions_set1), demand_nodes_2, ref(object_func2));

		thread t3(Pthread_learning, g, 2, agent_num, ref(last_dest_3), epsilon, alpha, gamma, lambda, ref(T_pi_3), ref(unexe_actions_set1), demand_nodes_3, ref(object_func3));

		thread t4(Pthread_learning, g, 3, agent_num, ref(last_dest_4), epsilon, alpha, gamma, lambda, ref(T_pi_4), ref(unexe_actions_set1), demand_nodes_4, ref(object_func4));

		thread t5(Pthread_learning, g, 4, agent_num, ref(last_dest_5), epsilon, alpha, gamma, lambda, ref(T_pi_5), ref(unexe_actions_set1), demand_nodes_5, ref(object_func5));

		t1.join();
		t2.join();
		t3.join();
		t4.join();
		t5.join();

		//state_label = Initial_State1(g, T_stage, actions_set_1, actions_set_2, actions_set_3, total_cost_1, total_cost_2, total_cost_3, last_dest_1, last_dest_2, last_dest_3);
		//Initial_State2(g, unexe_actions_set1, 5);
		if (unexe_actions_set1.empty()) break;
	}
	object_func = object_func1 + object_func2 + object_func3 + object_func4 + object_func5;
	//cout << "object_func =" << object_func << endl;

	if (object_func < best_obj)
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

		local_opt_store.clear();
		local_opt_store1.clear();
		local_opt_store2.clear();
		local_opt_store3.clear();
		local_opt_store4.clear();
		local_opt_store5.clear();

		local_opt_store = temp_s_a_store;
		local_opt_store1 = temp_s_a_store1;
		local_opt_store2 = temp_s_a_store2;
		local_opt_store3 = temp_s_a_store3;//Store best action so far, used to update optimal action set
		local_opt_store4 = temp_s_a_store4;
		local_opt_store5 = temp_s_a_store5;//Store best action so far, used to update optimal action set

		best_obj = object_func;
	}
	return object_func;
	//cout << "time_state.size() = " << time_state.size() << endl;
}

void multi_learning1(Graph g, vector<vector<int>> nodeConnection, vector<set<int>> actions_set, int& T_stage, int& record, int agent_num, double f_clock, double epsilon, double alpha, double gamma, double lambda, double& best_obj1, double& best_obj2, double& best_obj3, double& best_obj4, double& best_obj5, bool& best)
{
	best = false;
	bool flag = false;
	int imax = 0;
	if (agent_num == 3) imax = 150;
	else imax = 250;
	temp_s_a_store_rescue.clear();

	temp_s_a_store6.clear();
	temp_s_a_store7.clear();
	temp_s_a_store8.clear();
	temp_s_a_store9.clear();
	temp_s_a_store10.clear();

	set<int> demand_nodes_set;
	demand_nodes_set.clear();
	demand_nodes_set = demand_node_store;

	//set<int> new_connect_set;//Record newly connected points each round

	vector<vector<string>> s_a_store;
	unexe_actions_set.clear();
	unexe_actions_set1.clear();
	unexe_actions_set = damaged_edge_store;//Store damaged edges
	unexe_actions_set1 = const_demand_node_store;
	double max1 = 0, max2 = 0, max3 = 0, max4 = 0, max5 = 0;
	vector<int> PI_1, PI_2, PI_3, PI_4, PI_5;

	vector<int>  T_pi_1, T_pi_2, T_pi_3, T_pi_4, T_pi_5;
	vector<string> s_a_1, s_a_2, s_a_3, s_a_4, s_a_5;
	double total_cost_1 = 0, total_cost_2 = 0, total_cost_3 = 0, total_cost_4 = 0, total_cost_5 = 0;
	int last_dest_1 = 0, last_dest_2 = 1, last_dest_3 = 2, last_dest_4 = 3, last_dest_5 = 4;
	set<int> actions_set_1, actions_set_2, actions_set_3, actions_set_4, actions_set_5;
	actions_set_1.clear();
	actions_set_2.clear();
	actions_set_3.clear();
	actions_set_4.clear();
	actions_set_5.clear();
	//cout << "demand_nodes_set = " << demand_nodes_set.size() << endl;
	//cout << " unexe_actions_set = " << unexe_actions_set.size() << endl;

	s_a_store.clear();
	double object_func_rescue = INF;
	double temp_func_rescue1 = 0, temp_func_rescue2 = 0, temp_func_rescue3 = 0, temp_func_rescue4 = 0, temp_func_rescue5 = 0;
	map<double, int> time_state1, time_state2, time_state3, time_state4, time_state5;
	//state_label_1 = Initial_State1(g, actions_set_1, total_cost_1, last_dest_1);
	//Initial_State(g, demand_nodes_set, 5);
	Pthread_learning1(g, 0, agent_num, last_dest_1, epsilon, alpha, gamma, lambda, T_pi_1, max1, demand_nodes_set, temp_func_rescue1, time_state1);
	Pthread_learning1(g, 1, agent_num, last_dest_2, epsilon, alpha, gamma, lambda, T_pi_2, max2, demand_nodes_set, temp_func_rescue2, time_state2);
	Pthread_learning1(g, 2, agent_num, last_dest_3, epsilon, alpha, gamma, lambda, T_pi_3, max3, demand_nodes_set, temp_func_rescue3, time_state3);
	Pthread_learning1(g, 3, agent_num, last_dest_4, epsilon, alpha, gamma, lambda, T_pi_4, max4, demand_nodes_set, temp_func_rescue4, time_state4);
	Pthread_learning1(g, 4, agent_num, last_dest_5, epsilon, alpha, gamma, lambda, T_pi_5, max5, demand_nodes_set, temp_func_rescue5, time_state5);

	s_a_1.clear();
	s_a_2.clear();
	s_a_3.clear();
	s_a_4.clear();
	s_a_5.clear();
	s_a_store.clear();

	if (temp_func_rescue1 < best_obj1) {
		flag = true;
		best_obj1 = temp_func_rescue1;
		agent_best_pi_6.clear();
		agent_best_pi_6 = T_pi_1;
		local_opt_store6.clear();
		local_opt_store6 = temp_s_a_store6;
		times1.clear();
		times1 = time_state1;
	}
	if (temp_func_rescue2 < best_obj2) {
		flag = true;
		best_obj2 = temp_func_rescue2;
		agent_best_pi_7.clear();
		agent_best_pi_7 = T_pi_2;
		local_opt_store7.clear();
		local_opt_store7 = temp_s_a_store7;
		times2.clear();
		times2 = time_state2;
	}
	if (temp_func_rescue3 < best_obj3) {
		flag = true;
		best_obj3 = temp_func_rescue3;
		agent_best_pi_8.clear();
		agent_best_pi_8 = T_pi_3;
		local_opt_store8.clear();
		local_opt_store8 = temp_s_a_store8;
		times3.clear();
		times3 = time_state3;
	}
	if (temp_func_rescue4 < best_obj4) {
		flag = true;
		best_obj4 = temp_func_rescue4;
		agent_best_pi_9.clear();
		agent_best_pi_9 = T_pi_4;
		local_opt_store9.clear();
		local_opt_store9 = temp_s_a_store9;
		times4.clear();
		times4 = time_state4;
	}
	if (temp_func_rescue5 < best_obj5) {
		flag = true;
		best_obj5 = temp_func_rescue5;
		agent_best_pi_10.clear();
		agent_best_pi_10 = T_pi_5;
		local_opt_store10.clear();
		local_opt_store10 = temp_s_a_store10;
		times5.clear();
		times5 = time_state5;
	}
	//cout << " : " << qvalue_store_6.size() << " " << qvalue_store_7.size() << " " << qvalue_store_8.size() << " " << qvalue_store_9.size() << " " << qvalue_store_10.size() << endl;
	if (object_func_rescue > best_obj1 + best_obj2 + best_obj3 + best_obj4 + best_obj5) {
		times.clear();
		times = time_state;
		object_func_rescue = best_obj1 + best_obj2 + best_obj3 + best_obj4 + best_obj5;
	}
	else flag = false; //Optimal solution updated
	//Global optimal solution unchanged
	if (flag) {
		T_stage = 0;
		times.clear();
		times = time_state;
	}
	else {
		T_stage++;
	}
	//cout << "object_func_rescue  =" << object_func_rescue << " : " << temp_func_rescue1 + temp_func_rescue2 + temp_func_rescue3 + temp_func_rescue4 + temp_func_rescue5 << endl;
	//cout << "T_stage = " << T_stage << endl;

	if (T_stage >= imax) {
		T_stage = 0;

		vector<double> maxmin;
		maxmin.push_back(max1);
		maxmin.push_back(max2);
		maxmin.push_back(max3);
		maxmin.push_back(max4);
		maxmin.push_back(max5);
		int max = 0, min = INF;
		int index1 = 0, index2 = 0;
		for (int i = 0; i < agent_num; i++) {
			if (maxmin[i] > max) {
				max = maxmin[i];
				index1 = i;
			}
			if (maxmin[i] < min) {
				min = maxmin[i];
				index2 = i;
			}
		}
		int node = 0;
		switch (index1) {
		case 0:node = last_dest_1; break;
		case 1:node = last_dest_2; break;
		case 2:node = last_dest_3; break;
		case 3:node = last_dest_4; break;
		case 4:node = last_dest_5; break;
		default:break;
		}
		if (count(nodeConnection[index2].begin(), nodeConnection[index2].end(), node)) {
			record++;
			actions_set[index1].erase(node);
			actions_set[index2].insert(node);
			//cout << "index1 = " << index1 << " index2 = " << index2 << max1 << " " << max2 << " " << max3 << " " << max4 << " " << max5 << endl;
		}
	}/**/
}

bool test(Graph g, int agent_num, vector<vector<int>>& nodeConnection) {
	//cout << "unexe_actions_set1.size() = " << unexe_actions_set1.size() << "temp_state.size() = " << temp_state.size() << "demand_node_store.size() = " << demand_node_store.size() << endl;
	for (set<Line>::iterator damaged_iter = damaged_edge_store.begin(); damaged_iter != damaged_edge_store.end(); ++damaged_iter) {
		g.setConnectionStatus(*damaged_iter, false);
	}
	for (auto iter = time_state.begin(); iter != time_state.end(); ++iter) g.setConnectionStatus(iter->second, true);
	set<int> rest_demand_nodes = demand_node_store;   //Initialize remaining demand nodes
	map<int, vector<double>> dis_array;
	//cout << "This is test111111111111" << endl;
	for (int index = 0; index < agent_num; ++index)
	{
		vector<double> dis_array_0;
		g.shortestDis(index, dis_array_0);
		dis_array[index] = dis_array_0;
		for (auto node_iter = demand_node_store.begin(); node_iter != demand_node_store.end(); ++node_iter) {
			if (dis_array_0[*node_iter] < INF) {
				nodeConnection[index].push_back(*node_iter);
			}
		}
	}
	//cout << "This is test222222222222" << endl;
	for (auto node_iter = demand_node_store.begin(); node_iter != demand_node_store.end(); ++node_iter)
	{
		int i = agent_node[*node_iter];
		map<int, double> max_allowed_dis_store;
		switch (i) {
		case 1:max_allowed_dis_store = max_allowed_dis_store_0; break;
		case 2:max_allowed_dis_store = max_allowed_dis_store_1; break;
		case 3:max_allowed_dis_store = max_allowed_dis_store_2; break;
		case 4:max_allowed_dis_store = max_allowed_dis_store_3; break;
		case 5:max_allowed_dis_store = max_allowed_dis_store_4; break;
		default:break;
		}
		if (dis_array[i - 1][*node_iter] < INF)
			rest_demand_nodes.erase(*node_iter);
	}
	//cout << "This is test333333333333" << endl;

	if (rest_demand_nodes.size() != 0) {
		cout << "rest_demand_nodes.size() = " << rest_demand_nodes.size() << " time_state.size() = " << time_state.size() << endl;
		return false;
	}
	return true;
}

//Specify training cycles, episode: cycles
void episode_multi_Q(Graph g, int episode, int& record, int agent_num, int f_clock, double epsilon, double alpha, double gamma, double lambda, double& best_obj, double& best_obj_rescue)
{
	double temp_epsilon = 0;
	int start_flag = 0;
	int start_label = 0;
	int threshold = (int)(episode * 2 / 3);
	initialize();
	bool best = false;
	int best_i = 0;
	int T_stage = 0;
	double object_func_rescue1 = INF, object_func_rescue2 = INF, object_func_rescue3 = INF, object_func_rescue4 = INF, object_func_rescue5 = INF;
	vector<vector<int>> nodeConnection(agent_num);
	vector<set<int>> actions_set(agent_num);
	for (auto iter = const_demand_node_store.begin(); iter != const_demand_node_store.end(); ++iter)
	{
		switch (agent_node[*iter]) {
		case 1:actions_set[0].insert(*iter); break;
		case 2:actions_set[1].insert(*iter); break;
		case 3:actions_set[2].insert(*iter); break;
		case 4:actions_set[3].insert(*iter); break;
		case 5:actions_set[4].insert(*iter); break;
		default:break;
		}
	}
	for (int i = 0; i < episode; ++i)
	{
		//cout << i + 1 << " = ";
		int ex_thersold = episode - (int)((episode - start_flag) * 1 / 3);

		//Initialize damaged paths to false

		/*First 2/3 training cycles select as diversely as possible, last 1/3 cycles converge to optimal*/
		if (i < threshold)
			temp_epsilon = 1 - epsilon;
		else
			temp_epsilon = epsilon;

		double object_func = multi_learning(g, agent_num, f_clock, temp_epsilon, alpha, gamma, lambda, best_obj, best);

		if (!test(g, agent_num, nodeConnection) || object_func == INF) continue;

		for (set<Line>::iterator damaged_iter = damaged_edge_store.begin(); damaged_iter != damaged_edge_store.end(); ++damaged_iter)
			g.setConnectionStatus(*damaged_iter, false);
		for (set<int>::iterator damaged_iter = demand_node_store.begin(); damaged_iter != demand_node_store.end(); ++damaged_iter)
			node_rescue[*damaged_iter] = false;

		multi_learning1(g, nodeConnection, actions_set, T_stage, record, agent_num, f_clock, temp_epsilon, alpha, gamma, lambda, object_func_rescue1, object_func_rescue2, object_func_rescue3, object_func_rescue4, object_func_rescue5, best);

		if (best) best_i = i;
	}
	time_state_1.clear();
	time_state_1.insert(times1.begin(), times1.end());
	time_state_1.insert(times2.begin(), times2.end());
	time_state_1.insert(times3.begin(), times3.end());
	time_state_1.insert(times4.begin(), times4.end());
	time_state_1.insert(times5.begin(), times5.end());
	best_obj_rescue = object_func_rescue1 + object_func_rescue2 + object_func_rescue3 + object_func_rescue4 + object_func_rescue5;
	cout << "best_obj_rescue = " << best_obj_rescue << endl;
	cout << " Repair schedule：" << times.size() << endl;
	for (auto iter = times.begin(); iter != times.end(); ++iter)
	{
		cout << iter->first << " ";
	}
	cout << endl;
	cout << " Transport schedule：" << time_state_1.size() << endl;
	for (auto iter = time_state_1.begin(); iter != time_state_1.end(); ++iter)
	{
		cout << iter->first << " ";
	}
	cout << "best_i = " << best_i << endl;
}
//Multi-threading + Dual Q
int main()
{
	vector<int> node_num_store = { 40 };
	vector<int> edge_num_store = { 60 };
	vector<double> demand_pro_store = { 0.6 };
	vector<double> damaged_pro_store = { 0.7 };

	vector<bool> changed_demand_node_flag_store = { false };
	vector<bool> changed_damaged_edge_flag_store = { true };

	map<int, int> episode_store =
	{
		{ 40, 2000 },
		{ 60, 2500 },
		{ 80, 3000 },
	};
	map<int, int> AGENT_NUM_store =
	{
		{ 40, 3 },
		{ 60, 5 },
		{ 80, 5 },
	};

	int GRAPH_NUM = 10;

	int CHANGE_GRAPH = 5;

	int TRAIN_TIME = 1;

	int f_clock = 150;

	double v_raito = 0.1;

	double epsilon = 0.9, alpha = 0.4, gamma = 0.2, lambda = 0.1;

	string common_path = "E:\\Test_EXP\\90 Road Network Cases\\";//the path of the road network dataset

	string common_path2 = "E:\\Test_EXP\\CMARL\\";//Default storage path for experimental results：
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
				cout << connect_path << endl;

				Graph g(node_num, v_raito);

				createGraph(g, connect_path, AGENT_NUM);

				for (int train_index = 0; train_index < TRAIN_TIME; ++train_index)
				{
					Initial_Graph();
					int record = 0;
					double best_obj = INF, best_obj_rescue = INF;
					int episode = episode_store[node_num];

					clock_t start_time = clock();

					episode_multi_Q(g, episode, record, AGENT_NUM, f_clock, epsilon, alpha, gamma, lambda, best_obj, best_obj_rescue);

					clock_t end_time = clock();

					char str[20];
					sprintf_s(str, "g%d_c%d_CMARL", graph_index, AGENT_NUM);
					string tempstring(str);
					string filename = tempstring + ".txt";

					string write_path = common_path2 + node_edge + demand_damage + to_string(AGENT_NUM) + "_agent\\" + "Data\\";

					ofstream outfile;
					outfile.open(write_path + filename, ofstream::app);
					outfile << "CMARL:" << connect_path << endl;
					outfile << "node_num = " << node_num << "edge_num = " << edge_num << ";" << demand_node_pro << "_" << damaged_edge_pro << endl;
					outfile << "/***********************CMARL result***********************/" << endl;
					outfile << "solution:" << endl;
					outfile << " Repair schedule：" << times.size() << endl;
					for (auto iter = times.begin(); iter != times.end(); ++iter)
					{
						outfile << iter->first << " ";
					}
					outfile << endl;
					outfile << " Transport schedule：" << time_state_1.size() << endl;
					for (auto iter = time_state_1.begin(); iter != time_state_1.end(); ++iter)
					{
						outfile << iter->first << " ,";
					}
					outfile << endl;
					for (auto iter = times1.begin(); iter != times1.end(); ++iter)
					{
						outfile << iter->first << " ";
					}
					outfile << endl;
					for (auto iter = times2.begin(); iter != times2.end(); ++iter)
					{
						outfile << iter->first << " ";
					}
					outfile << endl;
					for (auto iter = times3.begin(); iter != times3.end(); ++iter)
					{
						outfile << iter->first << " ";
					}
					outfile << endl;
					for (auto iter = times4.begin(); iter != times4.end(); ++iter)
					{
						outfile << iter->first << " ";
					}
					outfile << endl;
					for (auto iter = times5.begin(); iter != times5.end(); ++iter)
					{
						outfile << iter->first << " ";
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
					outfile << "best_obj =" << best_obj << endl;
					outfile << "best_obj_RESCUE =" << best_obj_rescue << endl;
					cout << "best_obj =" << best_obj << endl;
					cout << "best_obj_RESCUE =" << best_obj_rescue << endl;
					outfile << "record =" << record << endl;
					cout << "record =" << record << endl;
					outfile.close();

					char str2[20];
					sprintf_s(str2, "g%d_c%d_CMARL", graph_index, AGENT_NUM);
					string tempstring2(str2);
					string filename2 = tempstring2 + "-object.txt";

					ofstream outfile2;
					outfile2.open(write_path + filename2, ofstream::app);
					//outfile2 << "best_obj = " << best_obj << endl;
					outfile2 << best_obj_rescue << endl;
					outfile2.close();

					char str3[20];
					sprintf_s(str3, "g%d_c%d_CMARL", graph_index, AGENT_NUM);
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