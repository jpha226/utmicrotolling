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
#include "AgentGraph.h"

using namespace boost;
using namespace std;

int main(int argc, char **argv)
{
  
	AgentGraph g;
	g.Plan();
	g.Detect();
	g.Move();
	g.Resolve();
	g.Plan();
	return 0;
  
}
