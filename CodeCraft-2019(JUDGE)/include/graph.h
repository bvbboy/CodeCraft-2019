#ifndef GRAPH_H
#define GRAPH_H
#include <list>
#include <vector>
#include <map>
#include <unordered_map>
#include <unordered_set>
#include <assert.h>
#include "vertex.h"
#include "edge.h"
#include "car.h"
using namespace std;

typedef pair<pair<int, int>, double> PAIR;

struct Compare
{
    bool operator() (const PAIR &l, const PAIR &r) {
        if (l.second != r.second)
            return l.second < r.second;
        else return l.first < r.first;
    }
};

class Graph
{
public:
    Graph();

    void addVertex(Vertex v);
    void addEdge(Edge e);
    void addCar(Car c);

    void loadGraph();
    void loadGraphDirect();
    vector<int> generatePath(int start, int dest);
    bool checkStatus();
    void processStatus();
    bool schedule(int time);
    void scheduleNewCars(int time);
    void scheduleNewCars(int time, bool priorOn);
    void resetStatus();
    void reset();

    bool processChosenCarAtCross(Car *chosenCar, Edge *curEdge, Edge *nextEdge, int channelBefore);
    double getTotalOccupancyRate();
    double getUndeployRate();
    bool checkCarsStop();
    void getOptStepsForCar(Car *chosenCar);
    vector<int> getSlowCars();
    void updateAdjWeight();
    bool directConnected(int start, int dest);
    vector<int> getCarsForRoad(Edge *e);
    vector<int> createCarSequeue(int crIdx);
    void addNewCarsSingle(Edge* road, int time);

    // for debug
    int printUnfinished();
    void printTotalOccupy();
    void adjustPlanningTime();
    void printFinishInfo();

public:
    int numVetices;
    int numEdges;
    int numCars;
    int totalOccupy;
    double totalOccupancyRate;
    double undeployRate;
    bool firstTime;

    double occupancyLimit;

    map<int, Vertex> id2cross; //id to cross
    map<int, pair<Edge, Edge>> id2road; // id to road
    map<int, vector<Vertex>> adjList; // cross id to its adj cross
    unordered_map<int, vector<pair<int, int>>> adj; // cross id to its adj cross distance
    unordered_map<int, vector<pair<int, int>>> adjSlow; // map for slow car
    unordered_map<int, vector<pair<int, int>>> adjFast; //map for fast car

    map<pair<int,int>, int> cross2road; // cross id1 id2 to road
    map<int, Car> id2car; //id to car

    map<pair<int, int>, int> id2roadCount;
    map<int, vector<int>> planningTime2Cars;
    map<int, unordered_set<int>> from2to;
    map<int, int> mainDirCount;
    unordered_map<int, vector<pair<int, int>>> adjBackup;
    map<int, int> id2time;
    vector<pair<int, int>> waitingTime;

    unordered_map<int, unordered_map<int, vector<int>>> pathExist;
};


#endif // GRAPH_H
