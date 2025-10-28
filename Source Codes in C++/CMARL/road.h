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

# define INF 0x3f3f3f3f // 0x3f3f3f3f is often used as infinity
# define NE_INF -0x3f3f3f3f

#endif
using namespace std;

struct Line
{
	int src;
	int dest;

	bool operator< (const Line& line) const; // Custom comparison operator <, because it will be used in map(<Line>, double) later
	// As an ordered associative container, map will sort the keys (<Line>), which is equivalent to defining a custom sorting rule
	Line() {};
	Line(int m_src, int m_dest);
};

// Define the structure for graph edges
struct Edge
{
	int src; // Starting point of the edge

	int dest; // Ending point of the edge

	double weight;

	double weight_store;

	bool isConnect; // Connection status

	Edge(int m_src, int m_dest, double m_weight); // Constructor for struct Line: used to assign values to declared variables
};

// Define the graph class
class Graph
{
	int N; // Total number of nodes

	double Vc_d; // Connected road speed : damaged road speed

	double* Importance; // Importance of demand nodes

	double* RescueTime; // Rescue vehicle access time to demand points

	vector<vector<double>> repairTime; // Define a 2D array repairTime

	//bool *isDamaged;
public:
	list<Edge>* adj; // Store all adjacent edges and their weights for each node

	Graph(int m_N, double m_V); // Constructor for class Graph: used to assign values to declared variables

	int getN(); // Get the total number of nodes

	void setImportance(int i, double value);

	double getImportance(int i);

	void addEdge(int u, int v, double w); // Store the vertices and length of each edge

	double getWeight(Line l); // Get the length of the specified edge

	bool judgeConnectStatus(Line l); // Judge the connection status of the specified edge

	double setConnectionStatus(Line l, bool value); // Set the connection status of each edge

	void setRepairTime(Line l, double value);

	double getRepairTime(Line l);

	void printGraph(ofstream& outfile); // Write road network information to the specified file

	// Find the shortest distance to node src [Without considering the use of damaged paths]
	void shortestDis(int src, vector<double>& dis_store);

	// Find the shortest distance to node src [Considering the situation where damaged paths are passable => travel cost = road length / v (v>1)]
	void shortestDis_EX(int src, vector<double>& dis_store);

	// Find the shortest path to node src [Without considering the use of damaged paths]
	void shortestPath(int src, vector<double>& dis_store, vector<vector<int>>& all_path_store);

	// Find the shortest path to node src [Under fully connected road network conditions, find the shortest path]
	void shortestPath1(int src, vector<double>& dis_store, vector<vector<int>>& all_path_store);

	// Find the shortest path to node src [Considering the use of damaged paths => (road repair cost + path length)]
	void shortestPath2(int src, vector<double>& dis_store, vector<vector<int>>& all_path_store);

	// Find the shortest path to node src [Considering the situation where damaged paths are passable => travel cost = road length / v (v>1)]
	void shortestPath_EX(int src, vector<double>& dis_store, vector<vector<int>>& all_path_store);

	void storePath(int dest, vector<int>& path, vector<int>& temp_path_store); // Store the shortest path from src to dest

	void setRescueTime(int i, double value);

	double getRescueTime(int i);
};
#pragma once