CC=g++

mapf: main.cpp AgentGraph.cpp
	$(CC) main.cpp AgentGraph.cpp -o mapf
