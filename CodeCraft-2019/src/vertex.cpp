#include "../include/vertex.h"


Vertex::Vertex()
{

}

Vertex::Vertex(int i) : id(i)
{

}

void Vertex::addNeighbor(int r0, int r1, int r2, int r3)
{
    neighbors.push_back(r0);
    neighbors.push_back(r1);
    neighbors.push_back(r2);
    neighbors.push_back(r3);
}

vector<int> Vertex::checkOtherRoads(int curRoad, int dir)
{
    vector<int> res;
    vector<int> neigh=neighbors;
    int idxCur; //index of current road
    for (size_t i=0; i<neigh.size(); ++i) {
        if (neigh[i]==curRoad) {
            idxCur=i;
            break;
        }
    }
    if (dir==LEFT) {
        int checkIdx=(idxCur==0) ? 3 : idxCur-1;
        int checkRoad=neigh[checkIdx];
        res.push_back(checkRoad);
        return res;
    }
    else if (dir==RIGHT) {
        int checkIdx1=(idxCur==3) ? 0 : idxCur+1;
        int checkIdx2=(idxCur<=1) ? idxCur+2 : idxCur-2;
        int checkRoad1=neigh[checkIdx1];
        int checkRoad2=neigh[checkIdx2];
        res.push_back(checkRoad1);
        res.push_back(checkRoad2);
        return res;
    }
    return res;
}



