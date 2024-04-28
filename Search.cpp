#include "Search.h"
#include <queue>
#include <chrono>
#include <unordered_map>
#include <iostream>
#include <limits>
#include <cassert>
#include <vector>
#include <algorithm>


namespace std
{
    //this is needed to store a pair<int,int> in an associative container
    //such as unorered_set and unordered_map
	template<> struct hash<std::pair<int,int>>
	{
		std::size_t operator()( const std::pair<int,int> & p) const noexcept
		{
            hash<int> hasher;
			return hasher(p.first) ^ (hasher(p.second)<<1);

		}
	};
}

std::vector<std::pair<int,int>> Search::reconstruct(const std::unordered_map<std::pair<int,int>,std::pair<int,int>> &pathCache, const std::pair<int,int> &start){
	std::deque<std::pair<int,int>> nodes;
	auto node = start;//make copy
    //traverse path from goal to start
	nodes.push_front(node);
	while(true){
		auto parent=pathCache.find(node);
		if(parent==pathCache.end())break;
		nodes.push_front(parent->second);
		node=parent->second;
	}
    //revert path and return it
    std::vector<std::pair<int,int>> vec;;
    for(auto p:nodes){
        vec.push_back(p);
    }
    return vec;
}

std::vector<std::pair<int,int>> Search::BFS(const Map& map, std::pair<int,int> start, std::pair<int,int> goal){
    std::cout<<"===========================\nRunning BFS...\n";
	auto startTime = std::chrono::high_resolution_clock::now();

    //stores possible directions
    std::pair<int,int> dirs[]{{-1,0},{-1,1},{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1}};

    bool visited[map.h][map.w]{false};      //we'll just use a matrix og booleans to indicated if visited
    std::queue<std::pair<int,int>> OPEN;
    std::unordered_map<std::pair<int,int>,std::pair<int,int>> pathCache;    ////hashmap to reconstruct path: child -> parent

    OPEN.push(start);
    visited[start.first][start.second]=true;

    while(!OPEN.empty()){
		auto pos=OPEN.front();
		OPEN.pop();

		if(pos==goal){
			auto endTime = std::chrono::high_resolution_clock::now();
			int count=0;
            for(int i=0;i<map.h;i++){
                for(int j=0;j<map.w;j++){
                    if(visited[i][j])count++;
                }
            }
            std::cout<<"VISITED: "<<count<<std::endl;
			std::cout<<"OPEN: "<<OPEN.size()<<std::endl;
			std::cout<<"FOUND in "<<(endTime-startTime).count()/1000000.0<<"ms\n";
			return reconstruct(pathCache,pos);;
		}

		for(auto dir:dirs){
			auto child=pos; //copy the position

            //then move it
            child.first+=dir.first;
            child.second+=dir.second;

            //if illegal or visited, skip it
            if(child.first<0
            ||child.first>=map.h
            ||child.second<0
            ||child.second>=map.w
            ||map._map[child.first][child.second]!=0
            ||visited[child.first][child.second]
            ){
                continue;
            }

			OPEN.push(child);
			visited[child.first][child.second]=true;
			pathCache[child]=pos;
		}
	}
	std::cout<<"NOT FOUND!!!!\n";
    
    //let's just return start and goal to draw them
    std::vector<std::pair<int,int>> path;
    path.push_back(start);
    path.push_back(goal);
    return path;
}

double Search::heuristic(const std::pair<int,int>& current, const std::pair<int,int>& goal) {
    // Usaremos la distancia de Manhattan como heurÃstica
    return abs(current.first - goal.first) + abs(current.second - goal.second);
}

std::vector<std::pair<int,int>> Search::Astar(const Map& map, std::pair<int,int> start, std::pair<int,int> goal){

    double weight = 1;

    std::cout<<"===========================\nRunning Astar...\n";
    auto startTime = std::chrono::high_resolution_clock::now();

    // Stores possible directions
    std::pair<int,int> dirs[] = {{-1,0},{-1,1},{0,1},{1,1},{1,0},{1,-1},{0,-1},{-1,-1}};

    // Priority queue for A*
    std::priority_queue<std::pair<double, std::pair<int,int>>> OPEN;

    // Visited array to keep track of visited nodes
    bool visited[map.h][map.w] {false};

    // Cost map to keep track of the cost to reach each node
    std::unordered_map<std::pair<int,int>, double> costMap;

    // Parent map to reconstruct the path
    std::unordered_map<std::pair<int,int>, std::pair<int,int>> pathCache;

    // Initialize starting point
    OPEN.push({0, start});
    visited[start.first][start.second] = true;
    costMap[start] = 0;

    while (!OPEN.empty()) {
        auto [cost, pos] = OPEN.top();
        OPEN.pop();

        if (pos == goal) {
            auto endTime = std::chrono::high_resolution_clock::now();
            int count = 0;
            for (int i = 0; i < map.h; i++) {
                for (int j = 0; j < map.w; j++) {
                    if (visited[i][j]) count++;
                }
            }
            std::cout << "VISITED: " << count << std::endl;
            std::cout << "OPEN: " << OPEN.size() << std::endl;
            std::cout << "FOUND in " << (endTime - startTime).count() / 1000000.0 << "ms\n";
            return reconstruct(pathCache, pos);
        }

        for (auto dir : dirs) {
            auto child = pos;
            child.first += dir.first;
            child.second += dir.second;

            if (child.first < 0 || child.first >= map.h || child.second < 0 || child.second >= map.w ||
                map._map[child.first][child.second] != 0 || visited[child.first][child.second]) {
                continue;
            }

            // Calculate the cost to reach the child node
            double newCost = costMap[pos] + 1; // Assuming each step has a cost of 1

            // Update the cost if it's lower than the current cost or not initialized
            if (!costMap.count(child) || newCost < costMap[child]) {
                costMap[child] = newCost;
                double priority = newCost + weight * heuristic(child, goal); // Weighted A* heuristic calculation
                OPEN.push({-priority, child});
                visited[child.first][child.second] = true;
                pathCache[child] = pos;
            }
        }
    }

    std::cout<<"NOT FOUND!!!!\n";
    std::vector<std::pair<int,int>> path;
    path.push_back(start);
    path.push_back(goal);
    return path;
}