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

	numNodes = 9;
	numEdges = 12;
	numAgents = 3;

	// Init graph
	graph = new AgentGraph::Graph(numNodes);
	weightmap = get(edge_weight, *graph);
	for (int i=0; i<numNodes; i++) {
		if (i < numEdges / 2){
			edgeList.push_back(edge(i,i+3));
			weights.push_back(1);
		}
		if ((i + 1) % 3 != 0){
			edgeList.push_back(edge(i,i+1));
			weights.push_back(1);
		}
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
	graph = new Graph(numNodes);
	weightmap = get(edge_weight, *graph);

}

AgentGraph::~AgentGraph() {
	delete graph;
}

void AgentGraph::Plan() {

	bool foundPath = false;

	for (int a = 0; a < numAgents; a++) {

		vector<Graph::vertex_descriptor> p(num_vertices(*graph));
	        vector<cost> d(num_vertices(*graph));
	    
		vertex start = agentList[a].start;
		vertex goal = agentList[a].dest;

 	        try {
			// call astar named parameter interface
			astar_search
		        (*graph, start,
		         distance_heuristic<Graph, cost>
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
				if ( agent_paths[a1][t] == agent_paths[a2][t] ) {
					cout << "conflict between " << a1 << " and " << a2 << " at step " << t << endl;
					Conflict c;
					c.T = t;
					c.V = agent_paths[a1][t];
					c.a1 = a1;
					c.a2 = a2;
					conflictList.push_back(c);
				}
			}
		}
	}

}

void AgentGraph::Resolve() {

	bool WP = false;
	Conflict conflict;
	typedef graph_traits<Graph> GraphTraits;
	typename property_map<Graph, vertex_index_t>::type index = get(vertex_index, *graph);
	typename GraphTraits::out_edge_iterator in_i, in_end;
	typename GraphTraits::edge_descriptor e;
	int time, a1, a2;


	for (int i = 0; i < conflictList.size(); i++) {
		
		conflict = conflictList[i];
		vertex v = conflict.V;
		time = conflict.T;
		a1 = conflict.a1;
		a2 = conflict.a2;
		vertex src;		
		

		// Get Edges to penalize
		for (tie(in_i, in_end) = in_edges(v, *graph); in_i != in_end; ++in_i)
		{
			e = *in_i;
			src = source(e, *graph);
			if ( src == agent_paths[a1][time - 1] || src == agent_pahts[a2][time - 1] )
				// e could potentially be penalized

		}

	
	//tie(out_i, out_end) = out_edges(v, *graph);
	//e = *out_i;
	//put(edge_weight, *graph, e,2);
	//	cout << "Weight: " << get(edge_weight,*graph,e) << endl;

	}

	vertex v = agent_paths[0][0];

        index = get(vertex_index, *graph);

	cout << "out-edges: ";
	for (tie(out_i, out_end) = out_edges(v, *graph); out_i != out_end; ++out_i) {
	        e = *out_i;
        	vertex src = source(e, *graph), targ = target(e, *graph);
	        cout << "(" << index[src] << "," << index[targ] << ") ";
      }
      cout << endl;


}

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
