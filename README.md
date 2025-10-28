# Integrated Scheduling and Routing of Repair Crews and Relief Vehicles via Collaborative Multi-Agent Reinforcement Learning
The source codes of the compared methods and the raw road network cases.

# The C++ Codes for the Compared Four Algorithms
Note that all code files (*.h and *.cpp) in the folder "Source Codes in C++" are compiled using Visual Studio 2022 or a later version.

BD: Lakzaei S, Rahmani D, Tosarkani B M,  Nasiri S (2023) Integrated optimal scheduling and routing of repair crew and relief vehicles after disaster: a novel hybrid solution approach. Annals of Operations Research 328(2): 1495--1522.

DP: Duque P A M, Dolinskaya I S, Sorensen K (2016) Network repair crew scheduling and routing for emergency relief distribution problem. European Journal of Operational Research 248(1): 272--285.

ACO: Shin Y, Kim S, Moon I (2019) Integrated optimal scheduling of repair crew and relief vehicle after disaster. Computers and Operations Research 105: 237--247.

CMARL: Integrated Scheduling and Routing of Repair Crews and Relief Vehicles via Collaborative Multi-Agent Reinforcement Learning

# 90 Road Network Cases：
When you test the algorithm, please copy the whole folder "90 Road Network Cases" to your test directory, such as "E:\Test_EXP\"

# Default Storage Directory for Experimental Results：
When you test the algorithm, please use the directory suggested on the Githup.

For example: When you test DP, copy the whole DP folder in "Default Storage Directory for Experimental Results" to your test directory, such as "E:\Test_EXP\"

# Additional Notes:
The program will output three txt files in the current directory where the project was created:

1.g[Road Network Case Index]_c[The Number of Agents]_[Algorithm Name].txt: This file saves the best-found solution by the algorithm.

2.g[Road Network Case Index]_c[The Number of Agents]_[Algorithm Name]-object.txt: This file saves the best objective function (f_u) value obtained by the algorithm.

3.g[Road Network Case Index]_c[The Number of Agents]_[Algorithm Name]-time.txt: This file saves the total running time consumed by the algorithm.

# How to Use Our Road Network Case
1.Every Road Network Case includs:

add_edge.txt: Set edges in the graph.

set_node.txt: Set nodes with severity weights.

set_repair.txt: Set repair time for damged edges.

[Agent Index]_max_dis.txt: Set the maximal acceptable distance for each node.

set_rescue.txt: Set the service time at each demand node.

agent_node1.txt: Set the location of each agent.

2.When you select a case, please update the following information in the .cpp file:

vector<int> node_num_store = {80};//The numnber of Nodes
vector<int> edge_num_store = {120};//The numnber of Edges
vector<double> demand_pro_store = {0.4};//The proportions of demand nodes |V_d|/|V|
vector<double> damaged_pro_store = {0.5};//The proportions of damaged edges |E_d|/|E|

3.Then, you can compile the code to test this case.
