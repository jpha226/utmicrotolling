#ifndef AGENTGRAPH_H
#define AGENTGRAPH_H

#include <boost/graph/astar_search.hpp>
#include <boost/graph/adjacency_list.hpp>
#include <boost/graph/random.hpp>
#include <boost/graph/graphviz.hpp>
#include <cstdio>
#include <sys/time.h>
#include <vector>
#include <list>
#include <iostream>
#include <fstream>
#include <math.h>
#include <string.h>

using namespace boost;

typedef float cost;

template<class Name, class LocMap>
class node_writer {
public:
  node_writer(Name n, LocMap l, float _minx, float _maxx,
              float _miny, float _maxy,
              unsigned int _ptx, unsigned int _pty)
    : name(n), loc(l), minx(_minx), maxx(_maxx), miny(_miny),
      maxy(_maxy), ptx(_ptx), pty(_pty) {}
  template <class Vertex>
  void operator()(std::ostream& out, const Vertex& v) const {
    float px = 1 - (loc[v].x - minx) / (maxx - minx);
    float py = (loc[v].y - miny) / (maxy - miny);
    out << "[label=\"" << name[v] << "\", pos=\""
        << static_cast<unsigned int>(ptx * px) << ","
        << static_cast<unsigned int>(pty * py)
        << "\", fontsize=\"11\"]";
  }
private:
  Name name;
  LocMap loc;
  float minx, maxx, miny, maxy;
  unsigned int ptx, pty;
};

template <class WeightMap>
class time_writer {
public:
  time_writer(WeightMap w) : wm(w) {}
  template <class Edge>
  void operator()(std::ostream &out, const Edge& e) const {
    out << "[label=\"" << wm[e] << "\", fontsize=\"11\"]";
  }
private:
  WeightMap wm;
};

// euclidean distance heuristic
template <class Graph, class CostType>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
  distance_heuristic(Vertex goal)
  :  m_goal(goal) {}
  CostType operator()(Vertex u)
  {
//    CostType dx = m_location[m_goal].x - m_location[u].x;
//    CostType dy = m_location[m_goal].y - m_location[u].y;
    return 0;//::sqrt(dx * dx + dy * dy);
  }
private:
  //LocMap m_location;
  Vertex m_goal;
};

struct found_goal {}; // exception for termination

// visitor that terminates when we find the goal
template <class Vertex>
class astar_goal_visitor : public boost::default_astar_visitor
{
public:
  astar_goal_visitor(Vertex goal) : m_goal(goal) {}
  template <class Graph>
  void examine_vertex(Vertex u, Graph& g) {
    if(u == m_goal)
      throw found_goal();
  }
private:
  Vertex m_goal;
};

struct Agent{

	int id;
	int start;
	int dest;
	int movingCost;

};

/*typedef adjacency_list<listS, vecS, undirectedS, no_property,
                    property<edge_weight_t, cost> > mygraph_t;
typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
typedef mygraph_t::vertex_descriptor vertex;
typedef mygraph_t::edge_descriptor edge_descriptor;
typedef mygraph_t::vertex_iterator vertex_iterator;
typedef std::pair<int, std::pair<int, int> > edge;
*/
class AgentGraph {


	public:
		
		// Constructors
		AgentGraph();
		~AgentGraph();
		AgentGraph(int n, int m, int a);		

		// Algorithm methods
		void Plan();
		void Detect();
		void Resolve();
		void Move();
	
		typedef adjacency_list<listS, vecS, undirectedS, no_property,
                    property<edge_weight_t, cost> > mygraph_t;
		typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
		typedef mygraph_t::vertex_descriptor vertex;
		typedef mygraph_t::edge_descriptor edge_descriptor;
		typedef mygraph_t::vertex_iterator vertex_iterator;
		typedef std::pair< int, int > edge;

	private:

		int numAgents;
		int numNodes;	
		int numEdges;
		mygraph_t *graph;
		WeightMap weightmap;

		// Lists for agents
		std::vector <Agent> agentList;
		std::vector <std::vector <vertex> > agent_paths;
		std::vector <edge> edgeList;
		std::vector <int> weights;
};


#endif
