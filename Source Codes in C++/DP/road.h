#pragma once
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

    bool operator< (const Line& line) const; // Customized comparison operator <, because it will be used later in map(<Line>, double)
    // Map is an ordered associative container, meaning it will sort the keys (<Line>), which is equivalent to customizing the sorting rule.

    Line(int m_src, int m_dest);
};

// Define the structure of an edge in the graph.
struct Edge
{
    int src; // Declare the start node of the edge.
    int dest; // Declare the end node of the edge.
    double weight;
    double weight_store;
    bool isConnect; // Declare the connection status.

    Edge(int m_src, int m_dest, double m_weight); // Constructor for struct Line: used to assign values to declared variables.
};

// Define the Graph class.
class Graph
{
    int N; // Total number of nodes.
    double Vc_d; // Speed on connected road segments : Speed on damaged road segments
    double* Importance; // Importance of demand nodes.
    double* RescueTime; // Time when the rescue vehicle visits the demand point.
    vector<vector<double>> repairTime; // Define a 2D array repairTime.
    // bool *isDamaged;
public:
    list<Edge>* adj; // Store all adjacent edges and their weights for each node.

    Graph(int m_N, double m_V); // Constructor for class Graph: used to assign values to declared variables.

    int getN(); // Get the total number of nodes.

    void setImportance(int i, double value);
    double getImportance(int i);

    void addEdge(int u, int v, double w); // Store the vertices and length of each edge.

    double getWeight(Line l); // Get the length of the specified edge.
    bool judgeConnectStatus(Line l); // Judge the connection status of the specified edge.
    double setConnectionStatus(Line l, bool value); // Set the connection status of each edge.

    void setRepairTime(Line l, double value);
    double getRepairTime(Line l);

    void printGraph(ofstream& outfile); // Write the road network information to the specified file.

    // Find the shortest distance to point src [without considering using damaged paths]
    void shortestDis(int src, vector<double>& dis_store);

    // Find the shortest distance to point src [considering the case where damaged paths can be used => travel cost = road length / v (v>1)]
    void shortestDis_EX(int src, vector<double>& dis_store);

    // Find the shortest path to point src [without considering using damaged paths]
    void shortestPath(int src, vector<double>& dis_store, vector<vector<int>>& all_path_store);

    // Find the shortest path to point src [under fully connected road network conditions, find the shortest path]
    void shortestPath1(int src, vector<double>& dis_store, vector<vector<int>>& all_path_store);

    // Find the shortest path to point src [considering the case of using damaged paths => (repair cost + path length)]
    void shortestPath2(int src, vector<double>& dis_store, vector<vector<int>>& all_path_store);

    // Find the shortest path to point src [considering the case where damaged paths can be used => travel cost = road length / v (v>1)]
    void shortestPath_EX(int src, vector<double>& dis_store, vector<vector<int>>& all_path_store);

    void storePath(int dest, vector<int>& path, vector<int>& temp_path_store); // Store the shortest path from src to dest.

    void setRescueTime(int i, double value);
    double getRescueTime(int i);
};