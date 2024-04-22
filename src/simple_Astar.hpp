#pragma once
#include <iostream>
#include <vector>
#include <list>
#include <math.h>


const int normal_cost = 2;
const int diagonal_cost = 3;

struct point{
    int x,y;
    int F,G,H;
    point *parent;

    // constructor
    point():x(0), y(0), F(0), G(0), H(0), parent(NULL){}
    point(int _x, int _y) :x(_x), y(_y), F(0), G(0), H(0), parent(NULL){}
};

class Astar{


public:
    // use default constructor

    void initAstar(std::vector<std::vector<int>> & _grid); // init
    point *findPath(point &start, point &dest); // return true if path is found
    std::list<point *> getPath(point &start, point &dest);

    // return true if the next point is in the map and not an obstacle
    bool isValid(const point* curr, const point* next) const; 
    bool checkFootprintCollision(const point* next) const; 

    std::vector<point *> getSuccessor(point *any); // get all surrounding points
    point *isInList(const std::list<point *> &list, const point *any) const;

    int getG(point *start, point *any); // g: cost from start to point
    int getH(point *any, point *dest); // h: point to destination
    int getF(point *any);

    point *getLowestF();
private:
    std::vector<std::vector<int>> grid;

    std::list<point *> openList;
    std::list<point *> closedList;

    int footprintSize = 5;
};