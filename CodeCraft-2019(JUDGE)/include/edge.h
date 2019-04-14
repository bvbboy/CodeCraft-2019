#ifndef EDGE_H
#define EDGE_H

#include "car.h"
#include <vector>
#include <map>
#include <iostream>
#include <algorithm>

using namespace std;

enum Ocuppy {
    SAME = 0,
    ADD = 1,
    REMOVE = 2
};

enum Level {
    NONRIGHT = 0,
    NONLEFT = 1,
    NONFORWARD = 2,
    PRIRIGHT = 3,
    PRILEFT = 4,
    PRIFORWARD = 5
};

class Edge
{
public:
    Edge();
    Edge(int id, int from, int to, int length, int speed, int channels, bool isDup);

    void updateOccupy(Car* car, int operation, int prevChannel=-1); //update the id of cars on road
    Car* chooseCarForCheck(map<int, Car>& id2car); // choose a car for check (procesStatus)
    Car* chooseCarForOpreation(map<int, Car>& id2car); // (schdule)
    int getNumCars(); // get the # of cars on road
    Car* findPrevCar(Car *car, map<int, Car>& id2car); // find prev car of given one
    vector<int> getWaitingCars(int channel); // get the # of waiting cars in given channel
    bool addNewCar(Car *c); // update occupy and car state when a new car is schduled
    bool addNewPriorCar(Car* c, map<int, Car> &id2car);
    bool isFull(); // if occupy is full
    double getOccupancyRate();
    int getNumWaitingCars(map<int, Car>& id2car);
    void updatePriorList(map<int, Car>& id2car);
    void updateCarList(map<int, Car>& id2car);

public:
    int id;
    int from, to;
    int length;
    int speed;
    int channels;
    bool isDup;

    vector<vector<int>> occupy; //cars on the road
    vector<vector<int>> prevOccupy; //compare to occupy if deadlock happens
    int speedLevel;
    double occupancyRate; //0.XX
    vector<double> time2occupancy;
    bool full;
    vector<int> priorCarList;
    vector<int> carList;
    int priorLevel;

};

#endif // EDGE_H
