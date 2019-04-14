#ifndef VERTEX_H
#define VERTEX_H
#include <vector>
#include <queue>
#include "edge.h"
using namespace std;

class Vertex
{
public:
    Vertex();
    Vertex(int id);

    void addNeighbor(int r0, int r1, int r2, int r3);
    //check other roads when current car turns left or right, return relative road id
    vector<int> checkOtherRoads(int curRoad, int dir);
public:
    int id;
    vector<int> neighbors; // four directions: N,E,S,W
    queue<int> waitingQueue; // check next cross if waitingQueue is empty
};

#endif // VERTEX_H
