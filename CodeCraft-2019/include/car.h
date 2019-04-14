#ifndef CAR_H
#define CAR_H

#include<vector>
#include<climits>
#include<iostream>
#include<map>
#include<cmath>
using namespace std;

// list all the status of a car
enum Status {
    UNDEPLOYED = 0,
    READY = 1,
    WAITING = 2,
    STOP = 3,
    FINISH = 4
};

// list all the direction of a car
enum Direction {
    FORWARD = 1,
    LEFT = 2,
    RIGHT =3
};

enum Speed {
    SLOW = 0,
    MEDIUM = 1,
    FAST = 2
};

enum MainDir {
    North = 0,
    East = 1,
    South = 2,
    WEST =3
};


class Car
{
public:
    Car();
    Car(int id, int from, int to, int speed, int planTime, bool prior, bool preset);

    void moveForward(); // stay on the same road
    bool isFinished();
    void updateCurrentCar(Car *prevCar, int roadSpeed); // update the information of current car with respect to prev car
    bool passCross(int nextRoad,vector<vector<int>> occupy, int limitSpeed, map<int, Car> &id2Car); // can pass the cross or not
    bool updateNextRoad(vector<int> neighbors); // update on which road, next road, direction
    void moveToFront(); //move on the same road
    void moveToEnd(); //the finish move
    vector<int> getTransitionPoint(int numCross);

public:
    int id;
    int from, to;
    int speed;
    int planTime;
    bool priority;
    bool preset;
    int originalPlanTime;

    vector<int> path; // roads
    int onRoad;// on which road
    int inChannel; // in which channel
    int dis2Cross;// distance to next cross
    int maxSpeed;// actual speed=min(this.speed, edge.speed, distance to frontcar)
    int nextRoad; // next road
    int direction;// 1:forward 2:left 3:right
    int status;// 0:undeployed 1:ready 2:waiting 3:stop 4:finish
    int dis2Prev;// distance to prev car
    bool isChecked;// processStatus
    int speedLevel;
    vector<int> pathDirCount;
    int mainDir;
    int optSteps;
    int transitionPoint;
    int finishTime;
    int actualSteps;
    int priorLevel;

    static int countFinish;
    static int countStart;
    static int presetStart;
};


#endif // CAR_H
