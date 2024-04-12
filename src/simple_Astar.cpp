#include "simple_Astar.hpp"


void Astar::initAstar(std::vector<std::vector<int>> & _grid){
    grid = _grid;
    std::cout << "Astar initialized" << std::endl;
}

// g: cost from start to point
int Astar::getG(point *q, point *successor){
    
    // use of ? mark: condition ? result_if_true : result_if_false
    int nextG = (abs(q->x - successor->x) + abs(q->y - successor->y)) == 1 ? normal_cost : diagonal_cost;
    int parentG = successor->parent == NULL ? 0 : successor->parent->G;

    return nextG + parentG;


}

 // h: cost from point to destination
int Astar::getH(point *any, point *dest){
    
    // using heuristic: Euclidean Distance
    int h = sqrt((any->x - dest->x)*(any->x - dest->x) + (any->y - dest->y)*(any->y - dest->y))*diagonal_cost;
    return h;
}


int Astar::getF(point *any){

    return any->G + any->H;
}

// classic algo: find smallest value in a list
point *Astar::getLowestF(){
    
    if (!openList.empty()){

        auto idxPoint = openList.front();
        for(auto &temp : openList){
            if (temp->F < idxPoint->F){
                idxPoint = temp;
            }
        }
        return idxPoint;
    } else{
        return NULL;
    }

}


bool Astar::isValid(const point* curr, const point* next) const{

    if(next->x < 0 || next->x > grid.size() - 1 // out of bound
    || next->y < 0 || next->y > grid[0].size() - 1
    || grid[next->x][next->y] != 0 // obstacle
    || (next->x == curr->x && next->y == curr->y) // curr is next
    || isInList(closedList, next) // is in closedList
    ) {
        return false;
    } else if (grid[next->x][curr->y] == 0 && grid[curr->x][next->y] == 0) {
        // all clear on diagonal
        return true;
    } else {
        return false;
    }
}

std::vector<point *> Astar::getSuccessor(point *any){

    std::vector<point *> successorsVec;

    for (int tempX = any->x - 1; tempX <= any->x + 1; tempX++){
        for (int tempY = any->y - 1; tempY <= any->y + 1; tempY++){
            if (isValid(any, new point(tempX, tempY))){
                successorsVec.push_back(new point(tempX, tempY));
            }
        }
    }
    return successorsVec;
}

point *Astar::isInList(const std::list<point *> &list, const point *any) const{

    for (auto temp : list){
        if (temp->x == any->x && temp->y == any->y) {
            return temp;
        }
    }

    return NULL;
}

std::list<point *> Astar::getPath(point &start, point &dest){

    point *tail = findPath(start, dest);
    std::list<point *> path;
    while (tail) {
        path.push_front(tail);
        tail = tail->parent;

    }

    openList.clear();
    closedList.clear();
    
    return path;

}


point *Astar::findPath(point &start, point &dest){

    openList.push_back(new point(start.x, start.y));

    while (!openList.empty()){

        auto q = getLowestF();
        
        openList.remove(q);
        closedList.push_back(q);
        auto successorsVec = getSuccessor(q);

        for (auto &successor : successorsVec){

            if (!isInList(openList, successor)) { // if not in openList

                successor->parent = q; // set successor parent to q

                successor->G = getG(q, successor);
                successor->H = getH(successor, &dest);
                successor->F = getF(successor);

                openList.push_back(successor);

            } else{

                int tempG = getG(q, successor);

                if (tempG < successor->G){ // found a closer path to successor

                    // update successor
                    successor->G = tempG;
                    successor->parent = q;
                    successor->F = getF(successor);
                }

            }

            point *resPoint = isInList(openList,&dest);
            if (resPoint){
                std::cout << "Astar: found path!" << std::endl;
                return resPoint;
            }

            
        }

        
    }
    std::cout << "Astar: planning failed!" << std::endl;
    return NULL;
}