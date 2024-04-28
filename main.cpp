#include "Map.h"
#include "Search.h"
#include "ColorMap.h"
#include <cassert>
#include <iostream>

int main(int argc, char *argv[]){
    assert(argc==6); 
          
    Map map(argv[1]);
    ColorMap colorMap(map);
    //colorMap.print();

    auto path = Search::BFS(map,{atoi(argv[2]),atoi(argv[3])},{atoi(argv[4]),atoi(argv[5])}); 
    //colorMap.print(path);
    
    
    float dist=0;
    for(int i=0;i<(int)path.size()-1;i++){
        int h= std::abs(path[i].first-path[i+1].first);
        int w= std::abs(path[i].second-path[i+1].second);
        w==h?dist+=1.41421:dist+=1;
    }
    std::cout<<"Path length: "<<dist<<std::endl;
    path = Search::Astar(map,{atoi(argv[2]),atoi(argv[3])},{atoi(argv[4]),atoi(argv[5])}); 
    //colorMap.print(path);
    dist=0;
    for(int i=0;i<(int)path.size()-1;i++){
        int h= std::abs(path[i].first-path[i+1].first);
        int w= std::abs(path[i].second-path[i+1].second);
        w==h?dist+=1.41421:dist+=1;
    }
    std::cout<<"Path length: "<<dist<<std::endl;
    return 0;
}