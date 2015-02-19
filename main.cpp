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


// auxiliary types
struct location
{
  float y, x; // lat, long
};
typedef float cost;

template <class Name, class LocMap>
class node_writer {
public:
  node_writer(Name n, LocMap l, float _minx, float _maxx,
              float _miny, float _maxy,
              unsigned int _ptx, unsigned int _pty)
    : name(n), loc(l), minx(_minx), maxx(_maxx), miny(_miny),
      maxy(_maxy), ptx(_ptx), pty(_pty) {}
  template <class Vertex>
  void operator()(ostream& out, const Vertex& v) const {
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
  void operator()(ostream &out, const Edge& e) const {
    out << "[label=\"" << wm[e] << "\", fontsize=\"11\"]";
  }
private:
  WeightMap wm;
};


// euclidean distance heuristic
template <class Graph, class CostType, class LocMap>
class distance_heuristic : public astar_heuristic<Graph, CostType>
{
public:
  typedef typename graph_traits<Graph>::vertex_descriptor Vertex;
  distance_heuristic(LocMap l, Vertex goal)
    : m_location(l), m_goal(goal) {}
  CostType operator()(Vertex u)
  {
    CostType dx = m_location[m_goal].x - m_location[u].x;
    CostType dy = m_location[m_goal].y - m_location[u].y;
    return ::sqrt(dx * dx + dy * dy);
  }
private:
  LocMap m_location;
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


int main(int argc, char **argv)
{
  
  // specify some types
  typedef adjacency_list<listS, vecS, undirectedS, no_property,
    property<edge_weight_t, cost> > mygraph_t;
  typedef property_map<mygraph_t, edge_weight_t>::type WeightMap;
  typedef mygraph_t::vertex_descriptor vertex;
  typedef mygraph_t::edge_descriptor edge_descriptor;
  typedef mygraph_t::vertex_iterator vertex_iterator;
  typedef std::pair<int, int> edge;
  
  int N = 10; // number of vertices
  int A = 3;
  // specify data
  char *name[N];
  
  for (int i=0; i< N; i++)
  {
	name[i] = new char[4];
	snprintf(name[i],sizeof(name[i]),"%d",i);
  }

  location locations[] = { // lat/long
    {0.0, 9.0}, {5.0, 7.0}, {10.0, 3.0}
  };
  edge edge_array[] = {
    edge(0,1), edge(1,2),
    edge(2,3), edge(3,4),
    edge(4,5), edge(5,6),
    edge(6,7), edge(7,8),
    edge(8,9), edge(9,0)
  };
  unsigned int num_edges = sizeof(edge_array) / sizeof(edge);
  int weights[num_edges];
  for ( int i=0; i<num_edges; i++)
	weights[i] = 1;	    
  // create graph
  mygraph_t g(N);
  WeightMap weightmap = get(edge_weight, g);
  for(std::size_t j = 0; j < num_edges; ++j) {
    edge_descriptor e; bool inserted;
    tie(e, inserted) = add_edge(edge_array[j].first,
                                edge_array[j].second, g);
    weightmap[e] = weights[j];
  }
  mt19937 gen(0);//time(0));
  bool foundPath = false;
  vector < vector <vertex> > agent_paths;
  for (int a = 0; a < A; a++)
  {

    // pick random start/goal
    vertex start = random_vertex(g, gen);
    vertex goal = random_vertex(g, gen);
  
  
    cout << "Start vertex: " << name[start] << endl;
    cout << "Goal vertex: " << name[goal] << endl;
  
//  ofstream dotfile;
//  dotfile.open("test-astar-cities.dot");
//  write_graphviz(dotfile, g,
  //               node_writer<char **, location*>
    //              (name, locations, 75.0, 78.0, 41.67, 44.0,
      //             1, 1),
        //         time_writer<WeightMap>(weightmap));
  
  
    vector<mygraph_t::vertex_descriptor> p(num_vertices(g));
    vector<cost> d(num_vertices(g));
    try {
    // call astar named parameter interface
      astar_search
        (g, start,
         distance_heuristic<mygraph_t, cost, location*>
          (locations, goal),
         predecessor_map(&p[0]).distance_map(&d[0]).
         visitor(astar_goal_visitor<vertex>(goal)));
  
  
    } catch(found_goal fg) { // found a path to the goal
      foundPath = true;
      vector <vertex> shortest_path;
      for(vertex v = goal;; v = p[v]) {
        shortest_path.push_back(v);
        if(p[v] == v)
          break;
      }
      if (foundPath){
      cout << "Shortest path from " << name[start] << " to "
           << name[goal] << ": ";
      vector<vertex>::iterator spi = shortest_path.begin();
      cout << name[start];
      for(++spi; spi != shortest_path.end(); ++spi)
        cout << " -> " << name[*spi];
      cout << endl << "Total travel time: " << d[goal] << endl;
      agent_paths.push_back(shortest_path);
      } else {
	cout << "Didn't find a path from " << name[start] << " to " << name[goal] << "!" << endl;
	}
    }
 
  // Detect conflicts
  for (int a1 = 0; a1 < A; a1++)
  {
    for (int a2 = a1; a2 < A; a2++)
    {
      for (int t=0; t < 2 t++);//min(agent_paths[a1].size(), agent_paths[a2].size()); t++)
      {
	//if ( agent_paths[a1][t] == agent_paths[a2][t] )
        	cout << "conflict between " << a1 << " and " << a2 << " at step " << t << endl;
      }
    }
  }
 
  }

  return 0;
  
}
