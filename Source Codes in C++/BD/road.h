
#ifndef _ROAD_H_
#define _ROAD_H_

#include<iostream>
#include<fstream>
#include<queue>
#include<vector>
#include<set>
#include<list>
#include<map>
#include<string>
#include <algorithm>
#include <sstream>

# define INF 0x3f3f3f3f // 0x3f3f3f3f is often used as infinity
# define NE_INF -0x3f3f3f3f

#endif
using namespace std;

struct Line {
	int src;
	int dest;
	// Default constructor
	Line() {
		src = 0;
		dest = 0;
	}
	bool operator< (const Line& line) const; // Customizes the comparison operator < because it will be used in map(<Line>,double)
	// Map is an ordered associative container, meaning it sorts the keys (<Line>), which requires defining a custom sorting rule.

	Line(int m_src, int m_dest);
};

// Define the structure for an edge in the graph
struct Edge {
	int src; // Declare the start node of the edge

	int dest; // Declare the end node of the edge

	double weight; // Current weight (can be INF if disconnected)

	double weight_store; // Original stored weight (always the fixed travel time/cost)

	bool isConnect; // Declare connection status

	Edge(int m_src, int m_dest, double m_weight); // Constructor for struct Edge: used to assign values to declared variables
};

// Define the Graph class
class Graph {
	int N; // Total number of nodes

	double* Importance; // Importance of demand nodes

	vector<vector<double>> repairTime; // Define a 2D array repairTime

	double* RescueTime; // Rescue vehicle arrival time at demand points

public:
	list<Edge>* adj; // Store all adjacent edges and their weights for each node

	Graph(int m_N);        // Constructor for class Graph: used to assign values to declared variables

	int getN(); // Get the total number of nodes

	void setImportance(int i, double value);

	double getImportance(int i);

	void addEdge(int u, int v, double w); // Store the vertices and length/weight of each edge

	double getWeight(Line l); // Get the length/weight of the specified edge

	bool judgeConnectStatus(Line l); // Check the connection status of the specified edge

	double setConnectionStatus(Line l, bool value); // Set the connection status for each edge

	void setRepairTime(Line l, double value);

	double getRepairTime(Line l);

	void printGraph(ofstream& outfile); // Write road network information to the specified file

	// Find the shortest distance to node src [without considering the use of damaged paths]
	void shortestDis(int src, vector<double>& dis_store);

	// Find the shortest path to node src [without considering the use of damaged paths]
	void shortestPath(int src, vector<double>& dis_store, vector<vector<int>>& all_path_store);

	// Find the shortest path to node src [with the road network fully connected, find the shortest path]
	void shortestPath1(int src, vector<double>& dis_store, vector<vector<int>>& all_path_store);

	// Find the shortest path to node src [considering the use of damaged paths => (road repair cost + path length)]
	void shortestPath2(int src, vector<double>& dis_store, vector<vector<int>>& all_path_store);

	void storePath(int dest, vector<int>& path, vector<int>& temp_path_store); // Store the shortest path from src to dest

	void setRescueTime(int i, double value);

	double getRescueTime(int i);
};