#include "car.h"


Car::Car()
{

}

Car::Car(int i, int f, int t, int s, int pt, bool prior, bool pre) :
    id(i), from(f), to(t), speed(s), planTime(pt), priority(prior), preset(pre)
{
    status=UNDEPLOYED;
    isChecked=false;
    pathDirCount.resize(4,0);
    optSteps=0;
    finishTime=INT_MAX;
    actualSteps=INT_MAX;
    priorLevel=0;
    originalPlanTime=planTime;
}

int Car::countFinish=0;
int Car::countStart=0;
int Car::presetStart=0;

void Car::moveForward()
{
    maxSpeed=min(maxSpeed,dis2Prev-1);
    dis2Prev -= maxSpeed;
    dis2Cross -= maxSpeed;
    status=STOP;
}

bool Car::isFinished()
{
    if (onRoad==path.back() && dis2Cross<=0) return true;
    else return false;
}

void Car::updateCurrentCar(Car *prevCar, int roadSpeed)
{
    if (!prevCar) dis2Prev=INT_MAX;
    else dis2Prev=dis2Cross-prevCar->dis2Cross;

    maxSpeed=min(speed,roadSpeed);
}

bool Car::passCross(int nextRoad, vector<vector<int> > occupy, int limitSpeed, map<int, Car> &id2Car)
{
    int m=occupy.size();
    int n=occupy[0].size();
    if (maxSpeed<=dis2Cross) { //cannot reach cross
        moveForward();
        return false;
    }
    else {
        maxSpeed=min(speed,limitSpeed); // speed of next road
        int disOverCross=maxSpeed-dis2Cross;
        if (disOverCross>0) {
            int count=disOverCross;
            int channelIdx=-1;
            for (int i=0; i<m; ++i) {
                if (occupy[i][n-1]==0) {
                    channelIdx=i;
                    break;
                }
            }
            if (channelIdx==-1) { // no extra room
                int prevCarId=occupy[m-1][n-1];
                Car *prevCar=&id2Car[prevCarId];
                if (prevCar->status==WAITING) {
                    return false;
                }
                else if (prevCar->status==STOP) {
                    moveToFront();
                    return false;
                }
            }
            else {
                for (int j=n-1; j>=n-count; --j) {
                    if(occupy[channelIdx][j]!=0) {
                        int prevCarId=occupy[channelIdx][j];
                        Car *prevCar=&id2Car[prevCarId];
                        if (prevCar->status==WAITING) {
                            return false;
                        }
                        else if (prevCar->status==STOP) {
                            inChannel=channelIdx;
                            dis2Cross=j+1;
                            dis2Prev=1;
                            onRoad=nextRoad;
                            maxSpeed=min(speed,limitSpeed);// =0?
                            status=STOP;
                            return true;
                        }
                    }
                }
                inChannel=channelIdx;
                dis2Cross=n-count;
                onRoad=nextRoad;
                maxSpeed=min(speed,limitSpeed);
                status=STOP;
                return true;
            }
        }
        else {
            moveToFront();
            return false;
        }
    }
}

bool Car::updateNextRoad(vector<int> neighbors)
{

    for (size_t i=0; i<path.size()-1; ++i) {
        if (onRoad==path[i]) {
            nextRoad=path[i+1]; // update nextRoad
            break;
        }
    }
    if (onRoad==nextRoad) { //on the last road
        direction=FORWARD;
        return true;
    }
    else {
        int in, out;
        for (size_t i=0; i<neighbors.size(); ++i) {
            if (neighbors[i]==onRoad) in=i;
            if (neighbors[i]==nextRoad) out=i;
        }
        int res=out-in; // update direction
             if (res==1 || res==-3) direction=LEFT;
        else if (res==2 || res==-2) direction=FORWARD;
        else if (res==3 || res==-1) direction=RIGHT;
        return false;
    }

}

void Car::moveToFront()
{
    dis2Cross=0;
    dis2Prev=INT_MAX;
    maxSpeed=speed; // min(speed,onRoad.speed)
    status=STOP;
}

void Car::moveToEnd()
{
    dis2Prev -= maxSpeed;
    dis2Cross -= maxSpeed;
    status=STOP;
    isChecked=false;
    if (isFinished()) {
        ++countFinish;
        status=FINISH;
//        cout<<"car No."<<id<<" finished!"<<" count = "<<countFinish<<endl;
    }
}

vector<int> Car::getTransitionPoint(int numCross)
{
    int crossNum=numCross;
    int sideLen=sqrt(crossNum);
    int candidate1=0, candidate2=0;
    int from_y=from%sideLen;
    int from_x=from/sideLen;
    if (from_y==0) from_y=8;
    else from_x++;
    int to_y=to%sideLen;
    int to_x=to/sideLen;
    if (to_y==0) to_y=8;
    else to_x++;

    int deltaY=to_y-from_y;
    if (deltaY==0) {
        candidate1=to;
    }
    else {
        candidate1=to-deltaY;
        candidate2=from+deltaY;
    }
    vector<int> res;
    res.push_back(candidate1);
    res.push_back(candidate2);
    return res;
}








