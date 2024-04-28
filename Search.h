#pragma once
#include <vector>
#include "Map.h"
#include <unordered_map>



class Search{

public:
    static std::vector<std::pair<int,int>> reconstruct(const std::unordered_map<std::pair<int,int>,std::pair<int,int>> &pathCache, const std::pair<int,int> &start);//reconstructs path
    static std::vector<std::pair<int,int>> BFS(const Map& map, std::pair<int,int> start, std::pair<int,int> goal);//finds and return path in map from start to goal
    static std::vector<std::pair<int,int>> Astar(const Map& map, std::pair<int,int> start, std::pair<int,int> goal);//finds and return path in map from start to goal
    static double Search::heuristic(const std::pair<int,int>& current, const std::pair<int,int>& goal);
};