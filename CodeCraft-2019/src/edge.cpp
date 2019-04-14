#include "../include/edge.h"
using namespace std;

Edge::Edge()
{

}

Edge::Edge(int i, int f, int t, int l,int s, int c, bool isD) :
    id(i), from(f), to(t), length(l), speed(s), channels(c), isDup(isD)
{
    occupy.resize(channels,vector<int>(length,0));
    prevOccupy=occupy;
    full=false;
    priorLevel=0;
}

void Edge::updateOccupy(Car *car, int operation, int prevChannel)
{
    if (operation==SAME) {
        int m=car->inChannel;
        for (int j=length-1; j>=0; --j) {
            if (occupy[m][j]==car->id) {
                occupy[m][j]=0; // clear
                break;
            }
        }
        occupy[m][car->dis2Cross]=car->id; //rewrite
    }
    else if (operation==REMOVE) {
        int m=prevChannel;
        for (int j=length-1; j>=0; --j) {
            if (occupy[m][j]==car->id) {
                occupy[m][j]=0; // clear
                break;
            }
        }
    }
    else if (operation==ADD) {
        int channel=car->inChannel;
        occupy[channel][car->dis2Cross]=car->id;
    }

    occupancyRate=getOccupancyRate();
}

Car *Edge::chooseCarForCheck(map<int, Car> &id2car)
{
    Car *c;
    for (int j=0; j<length;++j) {
        for (int i=0; i<channels; ++i) {
            if (occupy[i][j]!=0) {
                int carIdx=occupy[i][j];
                c=&id2car[carIdx];
                if (c->isChecked) continue;
                c->isChecked=true;
                return c;
            }
        }
    }
    return NULL;
}

Car *Edge::chooseCarForOpreation(map<int, Car> &id2car)
{

    vector<int> priorInChannel(channels, -1);
    for (int i=0; i<channels; ++i) {
        for (int j=0; j<length; ++j) {
            if (occupy[i][j]!=0) {
                int carIdx=occupy[i][j];
                Car *c=&id2car[carIdx];
                if (c->status==STOP) {
                    break;
                }
                else if (c->status==WAITING) {
                    if (c->priority) {
                        priorInChannel[i]=length-j;
                        break;
                    }
                    else {
                        break;
                    }
                }
            }
        }
    }
    int maxIdx=max_element(priorInChannel.begin(),priorInChannel.end())-priorInChannel.begin();
    int maxElement=priorInChannel[maxIdx];
    if (maxElement==-1) { // no prior car
        for (int j=0; j<length; ++j) {
            for (int i=0; i<channels; ++i) {
                if (occupy[i][j]!=0) {
                    int carIdx=occupy[i][j];
                    Car *c=&id2car[carIdx];
                    if (c->status==STOP) continue;
                    return c;
                }
            }
        }
        return NULL;
    }
    else {
        int carIdx=occupy[maxIdx][length-maxElement];
        Car *c=&id2car[carIdx];
        return c;
    }

}

int Edge::getNumCars()
{
    int nums=0;
    for (int j=0; j<length; ++j) {
        for (int i=0; i<channels; ++i) {
            if (occupy[i][j]!=0) nums++;
        }
    }
    return nums;
}

Car *Edge::findPrevCar(Car *car, map<int, Car> &id2car)
{
    if (occupy[car->inChannel][car->dis2Cross]!=car->id) {
        cout<<"ERROR : Cannot find car: "<<car->id<<" on road "<<id<<endl;
        return NULL;
    }
    for (int j=car->dis2Cross-1; j>=0; --j) {
        if (occupy[car->inChannel][j]!=0) {
            int prevCarNum=occupy[car->inChannel][j];
            return &id2car[prevCarNum];
        }
    }
    return NULL;
}

vector<int> Edge::getWaitingCars(int channel)
{
    vector<int> res;
    for (int j=0; j<length; ++j) {
        if (occupy[channel][j]!=0) {
            res.push_back(occupy[channel][j]);
        }
    }
    return res;
}

bool Edge::addNewCar(Car *c)
{
    int maxSpeed=min(c->speed,speed);
    int channelIdx=-1;
    for (int i=0; i<channels; ++i) {
        if (occupy[i][length-1]==0) {
            channelIdx=i;
            break;
        }
    }
    if (channelIdx==-1) {
        return false;
    }
    else {
        int dist=maxSpeed;
        for (int j=length-1; j>=length-maxSpeed; --j) {
            if (occupy[channelIdx][j]!=0) {
                dist=length-1-j;
                break;
            }
        }
        occupy[channelIdx][length-dist]=c->id;
        c->onRoad=c->path[0];
        c->nextRoad=c->path[0];
        c->dis2Cross=length-dist;
        c->inChannel=channelIdx;
        c->maxSpeed=maxSpeed;
        c->status=STOP;
        occupancyRate=getOccupancyRate();
        return true;
    }

}

bool Edge::addNewPriorCar(Car *c, map<int, Car> &id2car)
{
    int maxSpeed=min(c->speed,speed);
    int channelIdx=-1;
    for (int i=0; i<channels; ++i) {
        if (occupy[i][length-1]==0) {
            channelIdx=i;
            break;
        }
    }
    if (channelIdx==-1) {
        return false;
    }
    else {
        int dist=maxSpeed;
        for (int j=length-1; j>=length-maxSpeed; --j) {
            if (occupy[channelIdx][j]!=0) {
                int prevCarId=occupy[channelIdx][j];
                Car* prevCar=&id2car[prevCarId];
                if (prevCar->status==STOP) {
                    dist=length-1-j;
                    break;
                }
                else if (prevCar->status==WAITING) {
                    return false;
                }
            }
        }
        occupy[channelIdx][length-dist]=c->id;
        c->onRoad=c->path[0];
        c->nextRoad=c->path[0];
        c->dis2Cross=length-dist;
        c->inChannel=channelIdx;
        c->maxSpeed=maxSpeed;
        c->status=STOP;
        occupancyRate=getOccupancyRate();
        return true;
    }
}

bool Edge::isFull()
{
    for (int i=0; i<channels; ++i) {
        for (int j=0; j<length; ++j) {
            if (occupy[i][j]==0) {
               full=false;
               return full;
            }
        }
    }
    full=true;
    return full;
}

double Edge::getOccupancyRate()
{
    int n=getNumCars();
    occupancyRate=double(n)/double(length*channels);
    return occupancyRate;
}

int Edge::getNumWaitingCars(map<int, Car>& id2car)
{
    int nums=0;
    for (int j=0; j<length; ++j) {
        for (int i=0; i<channels; ++i) {
            if (occupy[i][j]!=0) {
                int carId=occupy[i][j];
                Car *c=&id2car[carId];
                if (c->status==WAITING) {
                    nums++;
                }

            }
        }
    }
    return nums;
}

void Edge::updatePriorList(map<int, Car>& id2car)
{
    vector<int> newPriorList;
    for (size_t i=0; i<priorCarList.size(); ++i) {
        int carId=priorCarList[i];
        Car* c=&id2car[carId];
        if (c->status!=UNDEPLOYED) continue;
        newPriorList.push_back(carId);
    }
    priorCarList.swap(newPriorList);
}

void Edge::updateCarList(map<int, Car> &id2car)
{
    vector<int> newCarList;
    for (size_t i=0; i<carList.size(); ++i) {
        int carId=carList[i];
        Car* c=&id2car[carId];
        if (c->status!=UNDEPLOYED) continue;
        newCarList.push_back(carId);
    }
    carList.swap(newCarList);
}


