#include"road.h"

// set automatically inserts elements in sorted order, but Line is a custom class and cannot be compared by default, so we need to define a custom comparison standard for Line
bool Line::operator< (const Line& line) const
{
	if (src < line.src || (src == line.src && dest < line.dest))
		return true;
	else
		return false;
}

// Constructor initialization
Line::Line(int m_src, int m_dest)
{
	if (m_src <= m_dest)
	{
		src = m_src;
		dest = m_dest;
	}
	else
	{
		src = m_dest;
		dest = m_src;
	}
}

// Constructor initialization, can also be written as follows:
// Edge::Edge(int m_src, int m_des, double m_weight)£ºsrc(m_src),des(m_des),weight(m_weight),isConnect(true){}
// Note: Cannot specify a return type for the constructor
Edge::Edge(int m_src, int m_dest, double m_weight)
{
	src = m_src;

	dest = m_dest;

	weight = m_weight;

	weight_store = m_weight;

	isConnect = true;
}

Graph::Graph(int m_N, double m_V) :repairTime(m_N, vector<double>(m_N))
{
	N = m_N;

	Vc_d = m_V;

	Importance = new double[m_N];

	RescueTime = new double[m_N];

	adj = new list<Edge>[m_N];

	//isDamaged = new bool[N];
}

int Graph::getN()
{
	return N;
}

void Graph::setImportance(int i, double value)
{
	Importance[i] = value;
}

double Graph::getImportance(int i)
{
	return Importance[i];
}

void Graph::addEdge(int u, int v, double w)
{
	adj[u].push_back(Edge(u, v, w)); // Insert the newly created Edge(u,v,w) into (list) adj[u]

	adj[v].push_back(Edge(v, u, w)); // Record this edge at both the head and tail nodes
}

double Graph::getWeight(Line l)
{
	for (list<Edge>::iterator it = adj[l.src].begin(); it != adj[l.src].end(); ++it)
	{
		if (it->dest == l.dest)
			return it->weight_store;
	}
}

bool Graph::judgeConnectStatus(Line l)
{
	for (list<Edge>::iterator it = adj[l.src].begin(); it != adj[l.src].end(); ++it)
	{
		if (it->dest == l.dest)
			return it->isConnect;
	}
}

double Graph::setConnectionStatus(Line l, bool value)
{
	double weight_store = 0;

	for (list<Edge>::iterator it = adj[l.src].begin(); it != adj[l.src].end(); ++it)
	{
		weight_store = it->weight_store;
		if (it->dest == l.dest)
			it->isConnect = value;

		if (it->isConnect)
			it->weight = it->weight_store;
		else
			it->weight = INF;
	}

	for (list<Edge>::iterator it = adj[l.dest].begin(); it != adj[l.dest].end(); ++it)
	{
		if (it->dest == l.src)
			it->isConnect = value;

		if (it->isConnect)
			it->weight = it->weight_store;
		else
			it->weight = INF;
	}

	return weight_store;
}

void Graph::setRepairTime(Line l, double value)
{
	repairTime[l.src][l.dest] = value;
}

double Graph::getRepairTime(Line l)
{
	return repairTime[l.src][l.dest];
}
void Graph::setRescueTime(int i, double value) {
	RescueTime[i] = value;
}

double Graph::getRescueTime(int i) {
	return RescueTime[i];
}

void Graph::printGraph(ofstream& outfile)
{
	for (int i = 0; i < N; ++i)
	{
		outfile << "Importance of" << i << ":" << getImportance(i) << endl;
		outfile << "Edge:";

		cout << "Importance of" << i << ":" << getImportance(i) << endl;
		cout << "Edge:";

		// 'auto' is a type specifier that infers the variable type from its initial value
		// 'iter' is an address pointer pointing to each element of adj[], and the variable type is complex, so 'auto' can be used
		for (auto iter = adj[i].begin(); iter != adj[i].end(); ++iter)
		{
			outfile << "src:" << iter->src << ";" << "dest:" << iter->dest << ";" << "weight:" << iter->weight << ";" << endl;

			cout << "src:" << iter->src << ";" << "dest:" << iter->dest << ";" << "weight:" << iter->weight << ";" << endl;
		}

		outfile << endl;
	}
}

/*
Debug test output
void Graph::printGraph2() {
for (int i = 0; i < N; ++i) {
//outfile << "Importance of" << i << ":" << getImportance(i) << endl;
//outfile << "Edge:";

cout << "Importance of" << i << ":" << getImportance(i) << endl;
cout << "Edge:";

// 'auto' is a type specifier that infers the variable type from its initial value
// 'iter' is an address pointer pointing to each element of adj[], and the variable type is complex, so 'auto' can be used
for (auto iter = adj[i].begin(); iter != adj[i].end(); ++iter) {
//outfile << "src:" << iter->src << ";" << "dest:" << iter->dest << ";" << "weight:" << iter->weight << ";" << endl;

cout << "src:" << iter->src << ";" << "dest:" << iter->dest << ";" << "weight:" << iter->weight << ";" << endl;
}

//outfile << endl;
}
}
*/

void Graph::shortestDis(int src, vector<double>& dis_store)
{
	// Create an empty set 'setds' to store processed nodes
	set< pair<double, int> > setds; // Each element in setds consists of a pair: 1. double: the shortest distance from the corresponding node to the source 'src'; 2. int: the corresponding node ID

	vector<double> dis(N, INF); // Create a double vector with N elements, all initialized to INF, to store the shortest distance from each node to the source

	setds.insert(make_pair(0, src)); // Set the distance from src to itself to 0, making src the starting point
	dis[src] = 0;

	// Loop until setds is empty
	while (!setds.empty())
	{
		pair<double, int> tmp = *(setds.begin()); // Store the first element of setds in a temporary variable and then remove it from setds

		setds.erase(setds.begin()); // Each iteration of the while loop removes the first element of setds

		int u = tmp.second; // Extract the node ID of the first element

		for (list<Edge>::iterator iter = adj[u].begin(); iter != adj[u].end(); ++iter)
		{
			int v = iter->dest; // Extract the ID of the node adjacent to node u

			if (dis[v] > dis[u] + iter->weight)
			{
				/*
				Purpose of the conditional statement:
				If the stored distance from src to v (dis[v]) is not infinity (INF), it means a path path1 connecting v was found in a previous traversal
				and path1 was stored as the shortest path distance in setds. But now a shorter distance is found (dis[v] > dis[u] + iter->weight),
				so we must first remove the original (dis[v], v) from setds, then update the new shortest distance.
				*/
				if (dis[v] != INF)
					setds.erase(setds.find(make_pair(dis[v], v)));
				// Note: The node removed from setds must have a defined finite distance. Otherwise, that node can never be reached.

				dis[v] = dis[u] + iter->weight;
				setds.insert(make_pair(dis[v], v));
			}
		}
	}

	for (int i = 0; i < N; ++i)
	{
		dis_store.push_back(dis[i]); // Store the minimum distance from each node to src in the specified array dis_store
	}
}

// Find the shortest distance to node src [Considering the use of damaged paths => (road repair cost + path length)]
void Graph::shortestDis_EX(int src, vector<double>& dis_store)
{
	// Create an empty set 'setds' to store processed nodes
	set< pair<double, int> > setds; // Each element in setds consists of a pair: 1. double: the shortest distance from the corresponding node to the source 'src'; 2. int: the corresponding node ID

	vector<double> dis(N, INF); // Create a double vector with N elements, all initialized to INF, to store the shortest distance from each node to the source
	vector<int> path(N, INF);

	setds.insert(make_pair(0, src)); // Set the distance from src to itself to 0, making src the starting point
	dis[src] = 0;

	// Loop until setds is empty
	while (!setds.empty())
	{
		pair<double, int> tmp = *(setds.begin()); // Store the first element of setds in a temporary variable and then remove it from setds

		setds.erase(setds.begin()); // Each iteration of the while loop removes the first element of setds

		int u = tmp.second; // Extract the node ID of the first element

		for (list<Edge>::iterator iter = adj[u].begin(); iter != adj[u].end(); ++iter)
		{
			int v = iter->dest; // Extract the ID of the node adjacent to node u

			// Consider the situation of using damaged nodes, calculate the cost of passing through the road
			double pass_cost = (iter->isConnect) ? iter->weight : double(iter->weight_store / Vc_d);

			if (dis[v] > dis[u] + pass_cost)
			{
				/*
				Purpose of the conditional statement:
				If the stored distance from src to v (dis[v]) is not infinity (INF), it means a path path1 connecting v was found in a previous traversal
				and path1 was stored as the shortest path distance in setds. But now a shorter distance is found (dis[v] > dis[u] + iter->weight),
				so we must first remove the original (dis[v], v) from setds, then update the new shortest distance.
				*/
				if (dis[v] != INF)
					setds.erase(setds.find(make_pair(dis[v], v)));
				// Note: The node removed from setds must have a defined finite distance. Otherwise, that node can never be reached.

				dis[v] = dis[u] + pass_cost;
				setds.insert(make_pair(dis[v], v));
			}
		}
	}

	for (int i = 0; i < N; ++i)
	{
		dis_store.push_back(dis[i]); // Store the minimum distance from each node to src in the specified array dis_store
	}
}

// Find the shortest path to node src [Without considering the use of damaged paths]
void Graph::shortestPath(int src, vector<double>& dis_store, vector<vector<int>>& all_path_store)
{
	// Create an empty set 'setds' to store processed nodes
	set< pair<double, int> > setds; // Each element in setds consists of a pair: 1. double: the shortest distance from the corresponding node to the source 'src'; 2. int: the corresponding node ID

	vector<double> dis(N, INF); // Create a double vector with N elements, all initialized to INF, to store the shortest distance from each node to the source
	vector<int> path(N, INF);

	setds.insert(make_pair(0, src)); // Set the distance from src to itself to 0, making src the starting point
	dis[src] = 0;

	// Loop until setds is empty
	while (!setds.empty())
	{
		pair<double, int> tmp = *(setds.begin()); // Store the first element of setds in a temporary variable and then remove it from setds

		setds.erase(setds.begin()); // Each iteration of the while loop removes the first element of setds

		int u = tmp.second; // Extract the node ID of the first element

		for (list<Edge>::iterator iter = adj[u].begin(); iter != adj[u].end(); ++iter)
		{
			int v = iter->dest; // Extract the ID of the node adjacent to node u

			if (dis[v] > dis[u] + iter->weight)
			{
				/*
				Purpose of the conditional statement:
				If the stored distance from src to v (dis[v]) is not infinity (INF), it means a path path1 connecting v was found in a previous traversal
				and path1 was stored as the shortest path distance in setds. But now a shorter distance is found (dis[v] > dis[u] + iter->weight),
				so we must first remove the original (dis[v], v) from setds, then update the new shortest distance.
				*/
				if (dis[v] != INF)
					setds.erase(setds.find(make_pair(dis[v], v)));
				// Note: The node removed from setds must have a defined finite distance. Otherwise, that node can never be reached.

				dis[v] = dis[u] + iter->weight;
				setds.insert(make_pair(dis[v], v));

				path[v] = u; // Store the path segment (u-v) as part of the shortest path (src-dest)
				// The path can be reconstructed using 'path'; e.g., path[1]=5 means node 5 is the previous node of node 1 in the shortest path
			}
		}
	}

	vector<int> temp_path_store; // Declare a temporary variable

	for (int i = 0; i < N; ++i)
	{
		dis_store.push_back(dis[i]); // Store the minimum distance from each node to src in the specified array dis_store

		temp_path_store.clear(); // Clear memory before use

		storePath(i, path, temp_path_store); // Store the shortest path from src to node i into the temporary array temp_path_store

		// all_path_store is responsible for storing the shortest paths from all nodes to src
		all_path_store.push_back(temp_path_store); // Store the shortest path from node i to src into the 2D array all_path_store
	}
}

// Find the shortest path to node src [Under fully connected road network conditions, find the shortest path]
void Graph::shortestPath1(int src, vector<double>& dis_store, vector<vector<int>>& all_path_store)
{
	// Create an empty set 'setds' to store processed nodes
	set< pair<double, int> > setds; // Each element in setds consists of a pair: 1. double: the shortest distance from the corresponding node to the source 'src'; 2. int: the corresponding node ID

	vector<double> dis(N, INF); // Create a double vector with N elements, all initialized to INF, to store the shortest distance from each node to the source
	vector<int> path(N, INF);

	setds.insert(make_pair(0, src)); // Set the distance from src to itself to 0, making src the starting point
	dis[src] = 0;

	// Loop until setds is empty
	while (!setds.empty())
	{
		pair<double, int> tmp = *(setds.begin()); // Store the first element of setds in a temporary variable and then remove it from setds

		setds.erase(setds.begin()); // Each iteration of the while loop removes the first element of setds

		int u = tmp.second; // Extract the node ID of the first element

		for (list<Edge>::iterator iter = adj[u].begin(); iter != adj[u].end(); ++iter)
		{
			int v = iter->dest; // Extract the ID of the node adjacent to node u

			// Shortest path under fully connected road network conditions
			double pass_cost = iter->weight_store;

			if (dis[v] > dis[u] + pass_cost)
			{
				/*
				Purpose of the conditional statement:
				If the stored distance from src to v (dis[v]) is not infinity (INF), it means a path path1 connecting v was found in a previous traversal
				and path1 was stored as the shortest path distance in setds. But now a shorter distance is found (dis[v] > dis[u] + iter->weight),
				so we must first remove the original (dis[v], v) from setds, then update the new shortest distance.
				*/
				if (dis[v] != INF)
					setds.erase(setds.find(make_pair(dis[v], v)));
				// Note: The node removed from setds must have a defined finite distance. Otherwise, that node can never be reached.

				dis[v] = dis[u] + pass_cost;
				setds.insert(make_pair(dis[v], v));

				path[v] = u; // Store the path segment (u-v) as part of the shortest path (src-dest)
				// The path can be reconstructed using 'path'; e.g., path[1]=5 means node 5 is the previous node of node 1 in the shortest path
			}
		}
	}

	vector<int> temp_path_store; // Declare a temporary variable

	for (int i = 0; i < N; ++i)
	{
		dis_store.push_back(dis[i]); // Store the minimum distance from each node to src in the specified array dis_store

		temp_path_store.clear(); // Clear memory before use

		storePath(i, path, temp_path_store); // Store the shortest path from src to node i into the temporary array temp_path_store

		// all_path_store is responsible for storing the paths from all nodes to src
		all_path_store.push_back(temp_path_store); // Store the shortest path from node i to src into the 2D array all_path_store
	}
}

// Find the shortest path to node src [Considering the use of damaged paths => (road repair cost + path length)]
void Graph::shortestPath2(int src, vector<double>& dis_store, vector<vector<int>>& all_path_store)
{
	// Create an empty set 'setds' to store processed nodes
	set< pair<double, int> > setds; // Each element in setds consists of a pair: 1. double: the shortest distance from the corresponding node to the source 'src'; 2. int: the corresponding node ID

	vector<double> dis(N, INF); // Create a double vector with N elements, all initialized to INF, to store the shortest distance from each node to the source
	vector<int> path(N, INF);

	setds.insert(make_pair(0, src)); // Set the distance from src to itself to 0, making src the starting point
	dis[src] = 0;

	// Loop until setds is empty
	while (!setds.empty())
	{
		pair<double, int> tmp = *(setds.begin()); // Store the first element of setds in a temporary variable and then remove it from setds

		setds.erase(setds.begin()); // Each iteration of the while loop removes the first element of setds

		int u = tmp.second; // Extract the node ID of the first element

		for (list<Edge>::iterator iter = adj[u].begin(); iter != adj[u].end(); ++iter)
		{
			int v = iter->dest; // Extract the ID of the node adjacent to node u

			// Consider the situation of using damaged nodes, calculate the cost of passing through the road
			double pass_cost = (iter->isConnect) ? iter->weight : iter->weight_store + repairTime[iter->src][iter->dest];

			if (dis[v] > dis[u] + pass_cost)
			{
				/*
				Purpose of the conditional statement:
				If the stored distance from src to v (dis[v]) is not infinity (INF), it means a path path1 connecting v was found in a previous traversal
				and path1 was stored as the shortest path distance in setds. But now a shorter distance is found (dis[v] > dis[u] + iter->weight),
				so we must first remove the original (dis[v], v) from setds, then update the new shortest distance.
				*/
				if (dis[v] != INF)
					setds.erase(setds.find(make_pair(dis[v], v)));
				// Note: The node removed from setds must have a defined finite distance. Otherwise, that node can never be reached.

				dis[v] = dis[u] + pass_cost;
				setds.insert(make_pair(dis[v], v));

				path[v] = u; // Store the path segment (u-v) as part of the shortest path (src-dest)
				// The path can be reconstructed using 'path'; e.g., path[1]=5 means node 5 is the previous node of node 1 in the shortest path
			}
		}
	}

	vector<int> temp_path_store; // Declare a temporary variable

	for (int i = 0; i < N; ++i)
	{
		dis_store.push_back(dis[i]); // Store the minimum distance from each node to src in the specified array dis_store

		temp_path_store.clear(); // Clear memory before use

		storePath(i, path, temp_path_store); // Store the shortest path from src to node i into the temporary array temp_path_store

		// all_path_store is responsible for storing the paths from all nodes to src
		all_path_store.push_back(temp_path_store); // Store the shortest path from node i to src into the 2D array all_path_store
	}
}

// Find the shortest path to node src [Considering the use of damaged paths => (road repair cost + path length)]
void Graph::shortestPath_EX(int src, vector<double>& dis_store, vector<vector<int>>& all_path_store)
{
	// Create an empty set 'setds' to store processed nodes
	set< pair<double, int> > setds; // Each element in setds consists of a pair: 1. double: the shortest distance from the corresponding node to the source 'src'; 2. int: the corresponding node ID

	vector<double> dis(N, INF); // Create a double vector with N elements, all initialized to INF, to store the shortest distance from each node to the source
	vector<int> path(N, INF);

	setds.insert(make_pair(0, src)); // Set the distance from src to itself to 0, making src the starting point
	dis[src] = 0;

	// Loop until setds is empty
	while (!setds.empty())
	{
		pair<double, int> tmp = *(setds.begin()); // Store the first element of setds in a temporary variable and then remove it from setds

		setds.erase(setds.begin()); // Each iteration of the while loop removes the first element of setds

		int u = tmp.second; // Extract the node ID of the first element

		for (list<Edge>::iterator iter = adj[u].begin(); iter != adj[u].end(); ++iter)
		{
			int v = iter->dest; // Extract the ID of the node adjacent to node u

			// Consider the situation of using damaged nodes, calculate the cost of passing through the road
			double pass_cost = (iter->isConnect) ? iter->weight : double(iter->weight_store / Vc_d);

			if (dis[v] > dis[u] + pass_cost)
			{
				/*
				Purpose of the conditional statement:
				If the stored distance from src to v (dis[v]) is not infinity (INF), it means a path path1 connecting v was found in a previous traversal
				and path1 was stored as the shortest path distance in setds. But now a shorter distance is found (dis[v] > dis[u] + iter->weight),
				so we must first remove the original (dis[v], v) from setds, then update the new shortest distance.
				*/
				if (dis[v] != INF)
					setds.erase(setds.find(make_pair(dis[v], v)));
				// Note: The node removed from setds must have a defined finite distance. Otherwise, that node can never be reached.

				dis[v] = dis[u] + pass_cost;
				setds.insert(make_pair(dis[v], v));

				path[v] = u; // Store the path segment (u-v) as part of the shortest path (src-dest)
				// The path can be reconstructed using 'path'; e.g., path[1]=5 means node 5 is the previous node of node 1 in the shortest path
			}
		}
	}

	vector<int> temp_path_store; // Declare a temporary variable

	for (int i = 0; i < N; ++i)
	{
		dis_store.push_back(dis[i]); // Store the minimum distance from each node to src in the specified array dis_store

		temp_path_store.clear(); // Clear memory before use

		storePath(i, path, temp_path_store); // Store the shortest path from src to node i into the temporary array temp_path_store

		// all_path_store is responsible for storing the paths from all nodes to src
		all_path_store.push_back(temp_path_store); // Store the shortest path from node i to src into the 2D array all_path_store
	}
}

void Graph::storePath(int dest, vector<int>& path, vector<int>& temp_path_store)
{
	if (path[dest] != INF)
	{
		storePath(path[dest], path, temp_path_store); // Recursively output each path segment

		temp_path_store.push_back(dest); // Store each path segment starting from dest into the specified array
		// These segments together form the shortest distance path (dest-->src)
	}
	else
	{
		temp_path_store.push_back(dest);
		return;
	}
	return;
}