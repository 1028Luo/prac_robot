#pragma once
#include <iostream>
#include <vector>
#include <list>
#include <math.h>
#include "simple_Astar.hpp"

// given a 2d map and a pose(start or dest), 
// return the corresponding map point on map
point pose2map(double poseX, double poseY){
    
    // depending on map
    double resolution = 0.05;
    double scailing = 1/resolution; 
    int x_offset = 284;
    int y_offset = 210;
    int map_idx_X =  round(scailing*poseX + x_offset);
    int map_idx_Y =  round(scailing*poseY + y_offset);

    return point(map_idx_X, map_idx_Y);

}

// reverse
int map2pose(){


    return 0;
}


void printMap(const std::vector<std::vector<int>>& map_temp) {
    for (const auto& row : map_temp) {
        for (int value : row) {
            std::cout << value << " ";
        }
        std::cout << std::endl;
    }
}