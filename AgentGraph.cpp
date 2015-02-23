#include "AgentGraph.h"
#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <cstdio>
#include <sys/time.h>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h>    // for sqrt
#include <string.h>

using namespace boost;
using namespace std;

AgentGraph::AgentGraph() {

	numNodes = 10;
	numEdges = 10;
	numAgents = 3;

	// Init graph
	graph = new AgentGraph::mygraph_t(numNodes);
	weightmap = get(edge_weight, *graph);
	for (int i=0; i<numEdges; i++) {
		if (i < numEdges - 1)
			edgeList.push_back(edge(i,i+1));
		else
			edgeList.push_back(edge(i,0));
		weights.push_back(1);
	}
	for(int j=0; j<numEdges; j++) {
		edge_descriptor e; bool inserted;	
		tie(e, inserted) = add_edge(edgeList[j].first, edgeList[j].second, *graph);
		weightmap[e] = weights[j];
	}

	// Init Agents
	mt19937 gen(time(0));
	for (int a = 0; a<numAgents; a++)
	{
		vertex start = random_vertex(*graph, gen);
		vertex goal = random_vertex(*graph, gen); 

		Agent temp;
		temp.id = a;
		temp.start = start;
		temp.dest = goal;
		temp.movingCost = 0;
		agentList.push_back(temp);
	}

}

AgentGraph::AgentGraph(int n, int m, int a) {

	numNodes = n;
	numAgents = a;
	graph = new mygraph_t(numNodes);
	weightmap = get(edge_weight, *graph);

}

AgentGraph::~AgentGraph() {
	delete graph;
}

void AgentGraph::Plan() {

	bool foundPath = false;

	for (int a = 0; a < numAgents; a++) {

		vector<mygraph_t::vertex_descriptor> p(num_vertices(*graph));
	        vector<cost> d(num_vertices(*graph));
	    
		vertex start = agentList[a].start;
		vertex goal = agentList[a].dest;

 	        try {
			// call astar named parameter interface
			astar_search
		        (*graph, start,
		         distance_heuristic<mygraph_t, cost>
		          (goal),
		         predecessor_map(&p[0]).distance_map(&d[0]).
		         visitor(astar_goal_visitor<vertex>(goal)));

		} catch(found_goal fg) { // found a path to the goal
			foundPath = true;
			vector <vertex> shortest_path;
			for(vertex v = goal;; v = p[v]) {
			        shortest_path.insert(shortest_path.begin(),v);
			        if(p[v] == v)
				        break;
			}
			if (foundPath){
				cout << "Shortest path from " << start << " to "
				<< goal << ": ";
				vector<vertex>::iterator spi = shortest_path.begin();
				cout << start;
				for(++spi; spi != shortest_path.end(); ++spi)
					cout << " -> " << *spi;
					cout << endl << "Total travel time: " << d[goal] << endl;
					agent_paths.push_back(shortest_path);
				} else {
					cout << "Didn't find a path from " << start << " to " << goal << "!" << endl;
			}

		}

	}
}

void AgentGraph::Detect() {

	// Detect conflicts
	for (int a1 = 0; a1 < numAgents - 1; a1++)
	{
		for (int a2 = a1 + 1; a2 < numAgents; a2++)
		{
			int length = min(agent_paths[a1].size(), agent_paths[a2].size());
			for (int t=0; t < length; t++)
			{
				if ( agent_paths[a1][t] == agent_paths[a2][t] )
					cout << "conflict between " << a1 << " and " << a2 << " at step " << t << endl;
			}
		}
	}

}

void AgentGraph::Resolve() {}

void AgentGraph::Move() {

	int max_length = 0;
	for (int a = 0; a < numAgents; a++) {
		if (agent_paths[a].size() > max_length) { 
			max_length = agent_paths[a].size();
		}
	}

	for (int t = 0; t < max_length; t++)
	{
		for (int a=0; a < numAgents; a++) {

			if (agent_paths[a].size() > t)
				agentList[a].currLocation = agent_paths[a][t];

		}
	}

}
