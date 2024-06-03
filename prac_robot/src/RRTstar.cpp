#include "RRTstar.hpp"


void RRTstar::initRRTstar(std::vector<std::vector<int>> & _grid){

    grid = _grid;


}

// main search function
std::vector<Point> RRTstar::search(){

    while (itr < itr_max){
        
        itr++;

        Node randomNode = genRandomNode();
        Node nearestNode = findNearest(randomNode);


        if(!checkObstacle(randomNode, nearestNode)){
            reachNode(randomNode);
            rewire();
        }


        if (reachedDest){
            return generatePath(lastNodeAdded);

        }

    }


}

// generates a random node
Node RRTstar::genRandomNode(){




}


Node RRTstar::findNearest(Node node){



}

float RRTstar::getDistance(const Point p, const Point q){



}

bool RRTstar::checkObstacle(const Node p, const Node q){




}

bool RRTstar::reachNode(const Node p){



}


bool RRTstar::rewire(){



}


bool RRTstar::reachedDest(){



}

std::vector<Point> generatePath(Node* node){



} 