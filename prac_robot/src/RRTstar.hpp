#pragma once
#include <iostream>
#include <vector>



const int normal_cost = 2;
const int diagonal_cost = 3;

struct Point{
    int x,y;


    // constructor
    Point(int _x, int _y) :x(_x), y(_y){}
};

struct Node{

    std::vector<Node*> children;
    Node* parent;
    Point position;
    float cost;

    Node(Point _position) :children(NULL), parent(NULL), position(_position), cost(0){}

};




class RRTstar{


public:
    // use default constructor

    void initRRTstar(std::vector<std::vector<int>> & _grid);

    // main search function
    std::vector<Point> search();

    // generates a random node
    Node genRandomNode();

    Node findNearest(Node node);

    float getDistance(const Point p, const Point q);

    bool checkObstacle(const Node p, const Node q);

    bool reachNode(const Node p);

    bool rewire();
    
    bool reachedDest();

    // given a node, return the path from root to this node
    std::vector<Point> generatePath(Node* node); 



private:

    std::vector<Node*> bestPath;
    Node* root;
    std::vector<std::vector<int>> grid;

    int itr_max;
    int itr;

    Node* lastNodeAdded;
    


};