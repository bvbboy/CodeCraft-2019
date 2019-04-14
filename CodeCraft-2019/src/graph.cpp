#include "../include/graph.h"
#include<algorithm>
#include<limits>
#include<unordered_map>

Graph::Graph()
{
    numEdges=0;
    numVetices=0;
    numCars=0;
    numPresetCars=0;
    totalOccupy=0.0;
    firstTime=true;

    occupancyLimit=0.08;
    threshold = 0.3;
}

void Graph::addVertex(Vertex v)
{
    id2cross[v.id]=v;
    ++numVetices;
}

void Graph::addEdge(Edge e)
{
    Vertex f=id2cross[e.from];
    Vertex t=id2cross[e.to];
    adjList[f.id].push_back(t);
//    adj[f.id].push_back(make_pair(t.id,e.length));
//    adj[f.id].push_back(make_pair(t.id,e.length/e.channels));
//    adj[f.id].push_back(make_pair(t.id,e.length/e.speed/e.channels)); // length/speed?
    cross2road[make_pair(f.id,t.id)]=e.id;
    if (e.isDup) {
        adjList[t.id].push_back(f);
        Edge e_rev=e;
        e_rev.from=e.to;
        e_rev.to=e.from;
        id2road[e.id]=make_pair(e, e_rev);
//        adj[t.id].push_back(make_pair(f.id,e.length));
//        adj[t.id].push_back(make_pair(f.id,e.length/e.channels));
//        adj[t.id].push_back(make_pair(f.id,e.length/e.speed/e.channels));
        cross2road[make_pair(t.id,f.id)]=e.id;
        ++numEdges;
    } else {
        Edge e_null(e.id,e.to,e.from,0,0,0,0);
        id2road[e.id]=make_pair(e,e_null);
    }
    ++numEdges;
}

void Graph::addCar(Car c)
{
    id2car[c.id]=c;
    ++numCars;
}

void Graph::loadGraph()
{
    // fill adj list
    for (auto &edges:id2road) {
        Edge *e1=&edges.second.first;
        Edge *e2=&edges.second.second;
        adj[e1->from].push_back(make_pair(e1->to,e1->length));
        adjBackup[e1->from].push_back(make_pair(e1->to,e1->length));
        if (e2->channels!=0) {
            adj[e2->from].push_back(make_pair(e2->to,e2->length));
            adjBackup[e2->from].push_back(make_pair(e2->to,e2->length));
        }
    }

    // generate initial path for non-preset cars
    for (auto &car:id2car) {
        Car *c=&car.second;
        if (c->preset==false) {
            vector<int> result=generatePath(c->from,c->to);
            c->path.push_back(result[0]);
        }
        else {

                allTime.push_back(c->planTime);
        }

    }
    sort(allTime.begin(),allTime.end());
    vector<int> nums;
    for (int i=0; i<allTime.size()-1; ++i) {
        if (allTime[i]!=allTime[i+1]) {
            nums.push_back(allTime[i]);
        }
    }
    nums.push_back(allTime.back());
    copy(nums.begin(),nums.end(),inserter(presetCarTime,presetCarTime.end()));

    // add car to each road's list
    for (auto &car:id2car) {
        Car *c=&car.second;
        int startId=c->from;
        int roadId=c->path[0];

        Edge *r=(id2road[roadId].first.from==startId) ? &id2road[roadId].first : &id2road[roadId].second;
        if (c->priority) r->priorCarList.push_back(c->id);
        else r->carList.push_back(c->id);
    }


    // for debug
//    {
//        int i=0;
//        for (auto &car:id2car) {
//            Car *c=&car.second;
//            cout << "path from "<<c->from<<" to "<<c->to<<" :"<<endl;
//            for (size_t j=0; j<c->path.size(); ++j) {
//                cout<<"->"<<c->path[j];
//            }
//            cout<<endl;
//            ++i;
//            if (i>=10) break;
//        }
//    }


}

void Graph::loadGraphDirect()
{

}

vector<int> Graph::generatePath(int start, int dest)
{
    if (!pathExist[start][dest].empty()) {
        vector<int> result = pathExist[start][dest];
        return result;
    }

    vector<int> path;
    vector<int> nodes;
    unordered_map<int, int> distances;
    unordered_map<int, int> previous;

    auto comparator = [&] (int left, int right) { return distances[left] > distances[right]; };

    unordered_map<int, vector<pair<int, int>>> map;
    map=adj;

    for (auto &vertex:map) {
        if (vertex.first==start) {
            distances[vertex.first]=0;
        } else {
            distances[vertex.first]=numeric_limits<int>::max();
        }
        nodes.push_back(vertex.first);
        push_heap(begin(nodes),end(nodes),comparator);
    }

    while (!nodes.empty()) {
        pop_heap(begin(nodes), end(nodes), comparator);
        int smallest=nodes.back();
        nodes.pop_back();
        if (smallest==dest) {
            while (previous.find(smallest)!=end(previous)) {
                path.push_back(smallest);
                smallest=previous[smallest];
            }
        }
        for (auto &neighbor : map[smallest]) {

            int alt = distances[smallest] + neighbor.second;
            if (alt<distances[neighbor.first]) {
                distances[neighbor.first] = alt;
                previous[neighbor.first] = smallest;
                make_heap(begin(nodes), end(nodes), comparator);
            }
        }
    }

    vector<int> result;
    int fromId=start;
    for (int j=path.size()-1; j>=0; --j) {
        int toId=path[j];
        int roadId=cross2road[pair<int,int>(fromId,toId)];
        result.push_back(roadId);
        fromId=toId;
    }
    pathExist[start][dest] = result;

    vector<int> dist;
    int totalDist=0;
    for (size_t i=0; i<result.size(); ++i) {
        int roadId = result[i];
        Edge *e = &id2road[roadId].first;
        totalDist += e->length;
    }
    dist.push_back(totalDist);
    distExist[start][dest] = dist;

    return result;
}

int Graph::lookAtDist(int start, int dest)
{
    if (!distExist[start][dest].empty()) {
        vector<int> result = distExist[start][dest];
        return result[0];
    }
    else {
        generatePath(start,dest);
        vector<int> result = distExist[start][dest];
        return result[0];
    }
}

int Graph::generateOnePath(int start, int dest)
{
    if (start==dest) return -1;
    if (!pathExist[start][dest].empty()) {
        return pathExist[start][dest][0];
    }
    else {
        vector<int> res = generatePath(start, dest);
        return res[0];
    }
}

bool Graph::checkStatus()
{
    // copy occupy to prevOccupy
    for (auto &edges:id2road) {
        Edge *e1=&edges.second.first;
        Edge *e2=&edges.second.second;
        if (e1->channels!=0) {
            e1->prevOccupy=e1->occupy;
        }
        if (e2->channels!=0) {
            e2->prevOccupy=e2->occupy;
         }
    }

    //return false if not all car finish
    for (auto &car:id2car) {
        Car *c=&car.second;
        if (c->status!=FINISH) {
            return false;
        }
    }
    return true;
}

// determine the state of cars
void Graph::processStatus()
{
    for (auto &road:id2road) {
        Edge *road1 = &road.second.first;
        Edge *road2 = &road.second.second;
        // find cars on road1
        int num1=road1->getNumCars();
        for (int i=0; i<num1; ++i) {
            Car *car=road1->chooseCarForCheck(id2car);
            Car *prevCar=road1->findPrevCar(car,id2car);
            car->updateCurrentCar(prevCar,road1->speed);
            //determine whether prev car exists
            if (prevCar==NULL) {
                // reach cross?
                if (car->dis2Cross >= car->maxSpeed) {
                    car->moveForward(); //stay on the same road
                    road1->updateOccupy(car, SAME);
                }
                else {
                    car->status=WAITING;
                }
            }
            else {
                if (prevCar->status==WAITING) {
                    if (car->maxSpeed<car->dis2Prev) {
                        car->moveForward();
                        road1->updateOccupy(car, SAME);
                    }
                    else {
                        car->status=WAITING;
                    }
                }
                else if (prevCar->status==STOP) {

                    car->moveForward(); //move to prev-1 location
                    road1->updateOccupy(car, SAME);
                }
            }
        }
        // if road2 is in valid
        if (road2->channels!=0) {
            // similar to road1
            int num2=road2->getNumCars();
            for (int i=0; i<num2; ++i) {
                Car *car=road2->chooseCarForCheck(id2car);
                Car *prevCar=road2->findPrevCar(car,id2car);
                car->updateCurrentCar(prevCar,road2->speed);
                if (prevCar==NULL) {
                    if (car->dis2Cross >= car->maxSpeed) {
                        car->moveForward();
                        road2->updateOccupy(car,SAME);
                    }
                    else {
                        car->status=WAITING;
                    }
                }
                else {
                    if (prevCar->status==WAITING) {
                        if (car->maxSpeed<car->dis2Prev) {
                            car->moveForward();
                            road2->updateOccupy(car, SAME);
                        }
                        else {
                            car->status=WAITING;
                        }
                    }
                    else if (prevCar->status==STOP) {
                        car->moveForward();
                        road2->updateOccupy(car, SAME);
                    }
                }
            }
        }
    }
}

// deal with WAITING cars
bool Graph::schedule(int time)
{
    int loops=0;
    int prevCountWaiting=printUnfinished(); // # of waiting cars
    if (prevCountWaiting==0) {
        return false;
    }
    int countWaiting=0;
    // while not all cars are in end state
    while (!checkCarsStop()) {
        loops++;
//        cout<<"LOOP: "<<loops<<endl;
        for (auto &cros:id2cross) {
            Vertex *cro = &cros.second;
            vector<int> edges=cro->neighbors;
            sort(edges.begin(),edges.end());
            for (auto edge : edges) {
                if (edge == -1) continue;
                Edge* r = (id2road[edge].first.to==cro->id) ? &id2road[edge].first : &id2road[edge].second;

                if (r->channels==0) continue;

                bool stayThisRoad=true;
                while (stayThisRoad) {
                    Car *c=r->chooseCarForOpreation(id2car);
                    if (!c) {
                        stayThisRoad=false;
                        break;
                    }

                    int channelBefore=c->inChannel;
                    bool result;

                    bool isFinal;
                    if (c->preset) {
                        isFinal = c->updateNextRoad(cro->neighbors);
                    }
                    else {
                        int idealRoad = generateOnePath(cro->id, c->to);
                        isFinal = updateNextRoadForCar(c, cro->id, idealRoad);
                    }

                    if (isFinal) {
                        Car *prevCar=r->findPrevCar(c,id2car);
                        c->updateCurrentCar(prevCar,r->speed);
                        if (prevCar==NULL) {
                            if (c->dis2Cross>=c->maxSpeed) {
                                c->moveForward();
                                r->updateOccupy(c,SAME);
                            }
                            else {
                                if (c->priority) {
                                    c->moveToEnd();
                                    totalOccupancyRate=getTotalOccupancyRate();
                                    c->finishTime = time;
                                    c->actualSteps = time - c->originalPlanTime;
                                    r->updateOccupy(c,REMOVE,c->inChannel);
                                }
                                else {
                                    int straightId;
                                    vector<int> neighbors=cro->neighbors;
                                    int idx=-1;
                                    for (size_t i=0; i<neighbors.size(); ++i) {
                                        if (neighbors[i]==r->id) {
                                            idx=i;
                                            break;
                                        }
                                    }

                                    straightId = (idx<2) ? idx+2 : idx-2;
                                    int straight=neighbors[straightId];
                                    if (straight==-1) {
                                        c->moveToEnd();
                                        totalOccupancyRate=getTotalOccupancyRate();
                                        c->finishTime = time;
                                        c->actualSteps = time - c->originalPlanTime;
                                        r->updateOccupy(c,REMOVE,c->inChannel);
                                    }
                                    else {
                                        Edge* strEdge = (id2road[straight].first.from==cro->id) ? &id2road[straight].first : &id2road[straight].second;
                                        if (strEdge->channels==0) {
                                            c->moveToEnd();
                                            totalOccupancyRate=getTotalOccupancyRate();
                                            c->finishTime = time;
                                            c->actualSteps = time - c->originalPlanTime;
                                            r->updateOccupy(c,REMOVE,c->inChannel);
                                        }
                                        else {
                                            // may conflict
                                            vector<int> crossCarList=createCarSequeue(cro->id);
                                            assert(!crossCarList.empty());
                                            bool conflict = false;
                                            for (size_t i=0; i<crossCarList.size(); ++i) {
                                                if (c->id==crossCarList[i]) continue;
                                                int otherId=crossCarList[i];
                                                Car *otherCar=&id2car[otherId];
                                                if (otherCar->nextRoad==straight) {
                                                    if (otherCar->priorLevel > c->priorLevel) {
                                                        conflict = true;
                                                        break;
                                                    }
                                                }
                                            }
                                            if (conflict) {
                                                stayThisRoad=false;
                                            }
                                            else {
                                                c->moveToEnd();
                                                totalOccupancyRate=getTotalOccupancyRate();
                                                c->finishTime = time;
                                                c->actualSteps = time - c->originalPlanTime;
                                                r->updateOccupy(c,REMOVE,c->inChannel);
                                            }


                                        }
                                    }
                                }
                            }
                        }
                        else {
                            // prev car cannot be in waiting state, so move to prev-1
                            c->moveForward();
                            r->updateOccupy(c,SAME);
                        }
                    }

                    else {

                        vector<int> crossCarList=createCarSequeue(cro->id);
                        assert(!crossCarList.empty());

                        Edge *nextEdge=(id2road[c->nextRoad].first.from==cro->id) ? &id2road[c->nextRoad].first : &id2road[c->nextRoad].second;
                        Car *prevCar=r->findPrevCar(c,id2car);
                        c->updateCurrentCar(prevCar,r->speed);

                        bool conflict = false;
                        for (size_t i=0; i<crossCarList.size(); ++i) {
                            if (c->id==crossCarList[i]) continue;
                            int otherId=crossCarList[i];
                            Car *otherCar=&id2car[otherId];
                            if (otherCar->nextRoad==c->nextRoad) {
                                if (otherCar->priorLevel > c->priorLevel) {
                                    conflict = true;
                                    break;
                                }
                            }
                        }
                        if (conflict) {
                            stayThisRoad=false;
                        }
                        else {
                            result=processChosenCarAtCross(c,r,nextEdge,channelBefore);
                            if (c->status==WAITING) {
                                stayThisRoad=false;
                            }
                        }

                    }


                    if (c->status==STOP) {
                        vector<int> cars=r->getWaitingCars(channelBefore);
                        for (size_t i=0; i<cars.size(); ++i) {
                            int carId = cars[i];
                            Car *car=&id2car[carId];
                            if (car->status==STOP) continue;
                            Car *prevCar=r->findPrevCar(car,id2car);
                            car->updateCurrentCar(prevCar,r->speed);

                            if (prevCar==NULL) {
                                // reach cross?
                                if (car->dis2Cross >= car->maxSpeed) {
                                    car->moveForward(); //stay on the same road
                                    r->updateOccupy(car, SAME);
                                }
                                else {
                                    car->status=WAITING;
                                }
                            }
                            else {
                                if (prevCar->status==WAITING) {
                                    if (car->maxSpeed<car->dis2Prev) {
                                        car->moveForward();//stay on the same road
                                        r->updateOccupy(car, SAME);
                                    }
                                    else {
                                        car->status=WAITING;
                                    }
                                }
                                else if (prevCar->status==STOP) {
                                    car->moveForward(); //move to prev-1 location
                                    r->updateOccupy(car, SAME);
                                }
                            }
                        }

                        addNewCarsSingle(r, time);

                    }

                }


            }



        }

        countWaiting = printUnfinished(); // for debug
        if (countWaiting!=prevCountWaiting) {
            prevCountWaiting=countWaiting;
        }
        else {
            cout<<"!!!!!!!!DEADLOCK!!!!!!!!!"<<endl;
            return true;
        }

    }


    cout<<"number of loops: "<<loops<<endl;
    return false;

}

// add new cars
void Graph::scheduleNewCars(int time, bool priorOn)
{
    double roadLimit=0.15;
    double finishRate=0.9;

    if (presetCarTime.find(time)!=presetCarTime.end()) {
        for (auto &edges:id2road) {

            Edge *e1=&edges.second.first;
            Edge *e2=&edges.second.second;
            if (!e1->priorCarList.empty()) {
                for (size_t i=0; i<e1->priorCarList.size(); ++i) {

                    int carId=e1->priorCarList[i];
                    Car* c=&id2car[carId];

                    if (c->planTime>time) continue;
                    if (c->preset) {
                        bool res=e1->addNewPriorCar(c, id2car);
                        if (res) {
                            c->planTime=time;
                            totalOccupancyRate=getTotalOccupancyRate();
//                            cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                            Car::countStart++;
                            Car::presetStart++;
                            e1->updatePriorList(id2car);
                            i--;
                        }
                    }

                }
            }

            if (e2->channels!=0) {
                if (!e2->priorCarList.empty()) {
                    for (size_t i=0; i<e2->priorCarList.size(); ++i) {

                        int carId=e2->priorCarList[i];
                        Car* c=&id2car[carId];

                        if (c->planTime>time) continue;
                        if (c->preset) {
                            bool res=e2->addNewPriorCar(c, id2car);
                            if (res) {
                                c->planTime=time;
//                                cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                totalOccupancyRate=getTotalOccupancyRate();
                                Car::countStart++;
                                Car::presetStart++;
                                e2->updatePriorList(id2car);
                                i--;
                            }
                        }

                    }
                }

            }

        }

        if (!priorOn) {

            for (auto &edges:id2road) {

                if (totalOccupancyRate >= occupancyLimit * numCars) {
                    break;
                }

                Edge *e1=&edges.second.first;
                Edge *e2=&edges.second.second;

                //add non-prior cars
                if (!e1->carList.empty()) {
                    for (size_t i=0; i<e1->carList.size(); ++i) {


                        int carId=e1->carList[i];
                        Car* c=&id2car[carId];
                        if (c->planTime>time) continue;
                        if (c->preset) {
                            bool res=e1->addNewCar(c);
                            if (res) {
                                c->planTime=time;
//                                cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                Car::countStart++;
                                Car::presetStart++;
                                totalOccupancyRate=getTotalOccupancyRate();
                                e1->updateCarList(id2car);
                                i--;
                            }
                        }

                    }
                }


                if (e2->channels!=0) {

                    //add non-prior cars
                    if (!e2->carList.empty()) {
                        for (size_t i=0; i<e2->carList.size(); ++i) {

                            int carId=e2->carList[i];
                            Car* c=&id2car[carId];
                            if (c->planTime>time) continue;
                            if (c->preset) {
                                bool res=e2->addNewCar(c);
                                if (res) {
                                    c->planTime=time;
//                                    cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                    Car::countStart++;
                                    Car::presetStart++;
                                    totalOccupancyRate=getTotalOccupancyRate();
                                    e2->updateCarList(id2car);
                                    i--;
                                }
                            }
                        }
                    }


                }

            }
        }

    }
    else {
        if (Car::countFinish>finishRate*numCars) {
            occupancyLimit=0.045;
            roadLimit=0.3;
            for (auto &car:id2car) {

                if (totalOccupancyRate >= occupancyLimit * numCars) {
                    break;
                }

                else {
                    Car* c = &car.second;

                    if (c->planTime<=time && c->status==UNDEPLOYED) {
                        int roadNum=c->path[0];
                        Edge *r=(id2road[roadNum].first.from==c->from) ? &id2road[roadNum].first : &id2road[roadNum].second;
                        if (r->channels==0) {
                            cout<<"ERROR: the edge of road "<<roadNum<<" does not exist"<<endl;
                        }

                        if (r->occupancyRate<roadLimit) {
                            bool res=r->addNewCar(c);
                            if (res) {
                                c->planTime=time;
//                                cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                Car::countStart++;
                                totalOccupancyRate=getTotalOccupancyRate();
                            }
                        }
                    }
                }
            }

        }

        else {
            // no preset car
            if (Car::presetStart == numPresetCars) {
                if (Car::countFinish > numCars*0.5) {
                    occupancyLimit=0.035;
                }
                else {
                    occupancyLimit=0.035;
                }

                for (auto &edges:id2road) {

                    if (totalOccupancyRate >= occupancyLimit) {
                        break;
                    }

                    Edge *e1=&edges.second.first;
                    Edge *e2=&edges.second.second;
                    if (!e1->priorCarList.empty()) {
                        for (size_t i=0; i<e1->priorCarList.size(); ++i) {

                            int carId=e1->priorCarList[i];
                            Car* c=&id2car[carId];

                            if (c->planTime>time) continue;

                            bool res=e1->addNewPriorCar(c, id2car);
                            if (res) {
                                c->planTime=time;
                                totalOccupancyRate=getTotalOccupancyRate();
//                                cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                Car::countStart++;
                                e1->updatePriorList(id2car);
                                i--;
                            }

                        }
                    }

                    if (e2->channels!=0) {
                        if (!e2->priorCarList.empty()) {
                            for (size_t i=0; i<e2->priorCarList.size(); ++i) {

                                int carId=e2->priorCarList[i];
                                Car* c=&id2car[carId];

                                if (c->planTime>time) continue;

                                bool res=e2->addNewPriorCar(c, id2car);
                                if (res) {
                                    c->planTime=time;
//                                    cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                    totalOccupancyRate=getTotalOccupancyRate();
                                    Car::countStart++;
                                    e2->updatePriorList(id2car);
                                    i--;
                                }

                            }
                        }

                    }

                }

                if (!priorOn) {
                    if (Car::countFinish > numCars*0.5) {
                        roadLimit=0.15;
                    }
                    else {
                        roadLimit=0.01;
                    }

                    for (auto &edges:id2road) {

                        if (totalOccupancyRate >= occupancyLimit * numCars) {
                            break;
                        }

                        Edge *e1=&edges.second.first;
                        Edge *e2=&edges.second.second;

                        //add non-prior cars
                        if (!e1->carList.empty()) {
                            for (size_t i=0; i<e1->carList.size(); ++i) {

                                if (totalOccupancyRate >= occupancyLimit * numCars) {
                                    break;
                                }

                                int carId=e1->carList[i];
                                Car* c=&id2car[carId];
                                if (c->planTime>time) continue;
                                if (e1->occupancyRate<roadLimit) {
                                    bool res=e1->addNewCar(c);
                                    if (res) {
                                        c->planTime=time;
//                                        cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                        Car::countStart++;
                                        totalOccupancyRate=getTotalOccupancyRate();
                                        e1->updateCarList(id2car);
                                        i--;
                                    }
                                }

                            }
                        }


                        if (e2->channels!=0) {

                            //add non-prior cars
                            if (!e2->carList.empty()) {
                                for (size_t i=0; i<e2->carList.size(); ++i) {

                                    if (totalOccupancyRate >= occupancyLimit * numCars) {
                                        break;
                                    }

                                    int carId=e2->carList[i];
                                    Car* c=&id2car[carId];
                                    if (c->planTime>time) continue;
                                    if (e2->occupancyRate<roadLimit) {
                                        bool res=e2->addNewCar(c);
                                        if (res) {
                                            c->planTime=time;
//                                            cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                            Car::countStart++;
                                            totalOccupancyRate=getTotalOccupancyRate();
                                            e2->updateCarList(id2car);
                                            i--;
                                        }
                                    }

                                }
                            }

                        }

                    }

                }



            }
            // has preset car
            else {
                occupancyLimit=0.03;
                for (auto &edges:id2road) {

                    if (totalOccupancyRate >= occupancyLimit * numCars) {
                        break;
                    }

                    Edge *e1=&edges.second.first;
                    Edge *e2=&edges.second.second;
                    if (!e1->priorCarList.empty()) {
                        for (size_t i=0; i<e1->priorCarList.size(); ++i) {

                            int carId=e1->priorCarList[i];
                            Car* c=&id2car[carId];

                            if (c->planTime>time) continue;
                            if (c->preset) {
                                bool res=e1->addNewPriorCar(c, id2car);
                                if (res) {
                                    c->planTime=time;
                                    totalOccupancyRate=getTotalOccupancyRate();
//                                    cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                    Car::countStart++;
                                    Car::presetStart++;
                                    e1->updatePriorList(id2car);
                                    i--;
                                }
                            }

                        }
                    }

                    if (e2->channels!=0) {
                        if (!e2->priorCarList.empty()) {
                            for (size_t i=0; i<e2->priorCarList.size(); ++i) {

                                int carId=e2->priorCarList[i];
                                Car* c=&id2car[carId];

                                if (c->planTime>time) continue;
                                if (c->preset) {
                                    bool res=e2->addNewPriorCar(c, id2car);
                                    if (res) {
                                        c->planTime=time;
//                                        cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                        totalOccupancyRate=getTotalOccupancyRate();
                                        Car::countStart++;
                                        Car::presetStart++;
                                        e2->updatePriorList(id2car);
                                        i--;
                                    }
                                }

                            }
                        }

                    }

                }

                if (!priorOn) {

                    for (auto &edges:id2road) {

                        if (totalOccupancyRate >= occupancyLimit * numCars) {
                            break;
                        }

                        Edge *e1=&edges.second.first;
                        Edge *e2=&edges.second.second;

                        //add non-prior cars
                        if (!e1->carList.empty()) {
                            for (size_t i=0; i<e1->carList.size(); ++i) {


                                int carId=e1->carList[i];
                                Car* c=&id2car[carId];
                                if (c->planTime>time) continue;
                                if (c->preset) {
                                    bool res=e1->addNewCar(c);
                                    if (res) {
                                        c->planTime=time;
//                                        cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                        Car::countStart++;
                                        Car::presetStart++;
                                        totalOccupancyRate=getTotalOccupancyRate();
                                        e1->updateCarList(id2car);
                                        i--;
                                    }
                                }

                            }
                        }


                        if (e2->channels!=0) {

                            //add non-prior cars
                            if (!e2->carList.empty()) {
                                for (size_t i=0; i<e2->carList.size(); ++i) {

                                    int carId=e2->carList[i];
                                    Car* c=&id2car[carId];
                                    if (c->planTime>time) continue;
                                    if (c->preset) {
                                        bool res=e2->addNewCar(c);
                                        if (res) {
                                            c->planTime=time;
//                                            cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                            Car::countStart++;
                                            Car::presetStart++;
                                            totalOccupancyRate=getTotalOccupancyRate();
                                            e2->updateCarList(id2car);
                                            i--;
                                        }
                                    }
                                }
                            }


                        }

                    }
                }

                if (totalOccupancyRate < 0.028 * numCars && totalOccupancyRate > 0.02 * numCars) {
                    if (!priorOn) {

                        roadLimit=0.01;
                        for (auto &edges:id2road) {

                            if (totalOccupancyRate >= occupancyLimit * numCars) {
                                break;
                            }

                            Edge *e1=&edges.second.first;
                            Edge *e2=&edges.second.second;

                            //add non-prior cars
                            if (!e1->carList.empty()) {
                                for (size_t i=0; i<e1->carList.size(); ++i) {

                                    if (totalOccupancyRate >= occupancyLimit * numCars) {
                                        break;
                                    }

                                    int carId=e1->carList[i];
                                    Car* c=&id2car[carId];
                                    if (c->planTime>time) continue;
                                    if (e1->occupancyRate<roadLimit) {
                                        bool res=e1->addNewCar(c);
                                        if (res) {
                                            c->planTime=time;
    //                                        cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                            Car::countStart++;
                                            totalOccupancyRate=getTotalOccupancyRate();
                                            e1->updateCarList(id2car);
                                            i--;
                                        }
                                    }

                                }
                            }


                            if (e2->channels!=0) {

                                //add non-prior cars
                                if (!e2->carList.empty()) {
                                    for (size_t i=0; i<e2->carList.size(); ++i) {

                                        if (totalOccupancyRate >= occupancyLimit * numCars) {
                                            break;
                                        }

                                        int carId=e2->carList[i];
                                        Car* c=&id2car[carId];
                                        if (c->planTime>time) continue;
                                        if (e2->occupancyRate<roadLimit) {
                                            bool res=e2->addNewCar(c);
                                            if (res) {
                                                c->planTime=time;
    //                                            cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                                Car::countStart++;
                                                totalOccupancyRate=getTotalOccupancyRate();
                                                e2->updateCarList(id2car);
                                                i--;
                                            }
                                        }

                                    }
                                }

                            }

                        }

                    }
                }
            }

        }


    }



}

void Graph::resetStatus()
{
    // set STOP cars into READY
    for (auto &car:id2car) {
        Car *c=&car.second;
        if (c->status==WAITING) {
            cout<<"ERROR: car id: "<<c->id<<" is still in WAITING STATUS"<<endl;
        }
        if (c->status==STOP) {
            c->isChecked=false;
            c->status=READY;
        }
    }

//    cout << "checking full roads..."<<endl;
//    for (auto &edges:id2road) {
//        Edge *e1=&edges.second.first;
//        Edge *e2=&edges.second.second;
//        if (e1->channels!=0) {
//            if (e1->isFull()) cout<<e1->id<<"from "<<e1->from<<" to "<<e1->to<<endl;
//        }
//        if (e2->channels!=0) {
//            if (e2->isFull()) cout<<e2->id<<"from "<<e2->from<<" to "<<e2->to<<endl;
//        }

//        double r1=e1->getOccupancyRate();
//        e1->time2occupancy.push_back(r1);
//        double r2=e2->getOccupancyRate();
//        e2->time2occupancy.push_back(r2);

//    }

    totalOccupancyRate=getTotalOccupancyRate();
    cout<<"totalOccupy: "<<totalOccupancyRate<<endl;

}

void Graph::reset()
{
    for (auto &car:id2car) {
        Car *c=&car.second;
        c->status=UNDEPLOYED;
        c->isChecked=false;
        c->optSteps=0;
        c->finishTime=INT_MAX;
        c->actualSteps=INT_MAX;
        c->pathDirCount.clear();
    }

    for (auto &cross:id2cross) {
        Vertex *v=&cross.second;
        while(!v->waitingQueue.empty()) {
            v->waitingQueue.pop();
        }
    }

    for (auto &edges:id2road) {
        Edge *e1=&edges.second.first;
        Edge *e2=&edges.second.second;

        e1->occupy.resize(e1->channels,vector<int>(e1->length,0));
        e1->prevOccupy=e1->occupy;
        e1->full=false;
        e1->time2occupancy.clear();

        if (e2->channels!=0) {
            e2->occupy.resize(e2->channels,vector<int>(e2->length,0));
            e2->prevOccupy=e2->occupy;
            e2->full=false;
            e2->time2occupancy.clear();
        }
    }

    firstTime=true;
    id2time.clear();
    waitingTime.clear();
    Car::countStart=0;
    Car::countFinish=0;
}

bool Graph::processChosenCarAtCross(Car *chosenCar, Edge *curEdge, Edge *nextEdge, int channelBefore)
{
    Car *prevCar=curEdge->findPrevCar(chosenCar, id2car);
    chosenCar->updateCurrentCar(prevCar,curEdge->speed);
    bool result=chosenCar->passCross(chosenCar->nextRoad,nextEdge->occupy,nextEdge->speed, id2car);
    if (result) {
        curEdge->updateOccupy(chosenCar,REMOVE,channelBefore);
        nextEdge->updateOccupy(chosenCar, ADD);
    }
    else curEdge->updateOccupy(chosenCar, SAME);
    return result;
}

int Graph::getTotalOccupancyRate()
{
    int carNum=0;
    carNum = Car::countStart - Car::countFinish;
//    totalOccupancyRate=double(carNum)/double(totalOccupy);
    return carNum;
}

double Graph::getUndeployRate()
{
    int carNum=0;
    for (auto &car:id2car) {
        Car *c=&car.second;
        if (c->status==UNDEPLOYED) {
            carNum++;
        }
    }
    undeployRate=double(carNum)/double(id2car.size());
    return undeployRate;
}

bool Graph::checkCarsStop()
{
    //return false if not all car stop
    for (auto &car:id2car) {
        Car *c=&car.second;
        if (c->isChecked==true) {
            if (c->status!=STOP) {
                return false;
            }
        }
    }
    return true;
}

void Graph::getOptStepsForCar(Car *chosenCar)
{
    int carSpeed=chosenCar->speed;
    vector<int> path=chosenCar->path;
    for (size_t i=0; i<path.size(); ++i) {
        int roadID=path[i];
        int roadSpeed=id2road[roadID].first.speed;
        int maxSpeed=min(carSpeed,roadSpeed);
        int roadLength=id2road[roadID].first.length;
        chosenCar->optSteps += roadLength/maxSpeed+1;
    }

}

vector<int> Graph::getSlowCars()
{
    vector<int> res;
    for (auto &car:id2car) {
        Car *c=&car.second;
        if (c->status==UNDEPLOYED && c->speedLevel==SLOW) {
            res.push_back(c->id);
        }
    }
    return res;
}

void Graph::updateAdjWeight()
{
    double addWeight=1.5;
    adj=adjBackup;
    for (auto &edges:id2road) {
        Edge *e1=&edges.second.first;
        Edge *e2=&edges.second.second;
        int fromId=e1->from;
        int toId=e1->to;
        int num=e1->getNumCars();
        int n=adj[fromId].size();
        for (int i=0; i<n; ++i) {
            if (toId==adj[fromId][i].first) {
                adj[fromId][i].second += num*addWeight;
                break;
            }
        }
        if (e2->channels!=0) {
            int fromId=e2->from;
            int toId=e2->to;
            int num=e2->getNumCars();
            int n=adj[fromId].size();
            for (int i=0; i<n; ++i) {
                if (toId==adj[fromId][i].first) {
                    adj[fromId][i].second += num*addWeight;
                    break;
                }
            }
        }
    }

}

bool Graph::directConnected(int start, int dest)
{
    vector<int> steps=generatePath(start,dest);
    int dist=0;
    if (abs(start-dest)>=8) {
        dist=abs(start-dest)/8;
    }
    else {
        dist=abs(start-dest);
    }
    if (steps.size()==dist) return true;
    else if (steps.size()>dist) return false;
    else {
        cout<<"error: impossible!!!!"<<endl;
        return false;
    }
}

vector<int> Graph::getCarsForRoad(Edge *e)
{
    vector<int> res;
    int fromId=e->from;
    int roadId=e->id;
    for (auto &car:id2car) {
        Car *c=&car.second;
        if (c->status==UNDEPLOYED && c->from==fromId && c->path[0]==roadId) {
            res.push_back(c->id);
        }
    }
    return res;
}

vector<int> Graph::createCarSequeue(int crIdx)
{
    vector<int> crossCarList;
    Vertex *v = &id2cross[crIdx];
    for (int roIdx : v->neighbors) {
        if (roIdx != -1) {
            Edge *r;
            if (id2road[roIdx].first.to==crIdx) {
                r=&id2road[roIdx].first;
            }
            else {
                r=&id2road[roIdx].second;
            }
            r->priorLevel=0;
        }
    }

    vector<int> edges = v->neighbors;
    sort(edges.begin(),edges.end());
    for (int roadIdx : edges) {
        if (roadIdx==-1) continue;
        Edge *r;
        if (id2road[roadIdx].first.to==crIdx) {
            r=&id2road[roadIdx].first;
        }
        else {
            r=&id2road[roadIdx].second;
        }

        if (r->channels==0) continue;

        Car *c=r->chooseCarForOpreation(id2car);
        if (c) {
            crossCarList.push_back(c->id);
            if (c->preset) {
                c->updateNextRoad(v->neighbors);
            }
            else {
                int idealRoad = generateOnePath(crIdx, c->to);
                updateNextRoadForCar(c,crIdx,idealRoad);
            }

            if (c->priority) {
                if (c->direction==FORWARD) c->priorLevel=PRIFORWARD;
                else if (c->direction==LEFT) c->priorLevel=PRILEFT;
                else if (c->direction==RIGHT) c->priorLevel=PRIRIGHT;
            }
            else {
                if (c->direction==FORWARD) c->priorLevel=NONFORWARD;
                else if (c->direction==LEFT) c->priorLevel=NONLEFT;
                else if (c->direction==RIGHT) c->priorLevel=NONRIGHT;
            }
        }
    }
    return crossCarList;
}

void Graph::addNewCarsSingle(Edge *road, int time)
{
    if (presetCarTime.find(time)!=presetCarTime.end()) {
        road->updatePriorList(id2car);
        if (!road->priorCarList.empty()) {
            for (size_t i=0; i<road->priorCarList.size(); ++i) {

                int carId=road->priorCarList[i];
                Car* c=&id2car[carId];

                if (c->planTime > time) continue;
                if (c->preset) {
                    bool res=road->addNewPriorCar(c, id2car);
                    if (res) {
                        c->planTime = time;
//                        cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                        Car::countStart++;
                        Car::presetStart++;
                        road->updatePriorList(id2car);
                        i--;
                    }
                }
            }
        }
    }

    else {
        double roadLimit;
        if (Car::presetStart==numPresetCars) {
            roadLimit=0.5;
        }
        else {
            if (totalOccupancyRate < 0.028 * numCars && totalOccupancyRate > 0.001 * numCars) {
                roadLimit=0.4;
            }
            else {
                roadLimit=0.4;
            }

        }

        road->updatePriorList(id2car);
        if (!road->priorCarList.empty()) {
            for (size_t i=0; i<road->priorCarList.size(); ++i) {

                int carId=road->priorCarList[i];
                Car* c=&id2car[carId];

                if (c->planTime > time) continue;
                if (c->preset) {
                    bool res=road->addNewPriorCar(c, id2car);
                    if (res) {
                        c->planTime = time;
//                        cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                        Car::countStart++;
                        Car::presetStart++;
                        road->updatePriorList(id2car);
                        i--;
                    }
                }
                else {
                    if (totalOccupancyRate >= occupancyLimit* numCars) {
                        break;
                    }
                    if (road->occupancyRate<roadLimit) {
                        bool res=road->addNewPriorCar(c, id2car);
                        if (res) {
                            c->planTime = time;
//                            cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                            Car::countStart++;
                            road->updatePriorList(id2car);
                            i--;
                        }
                    }

                }

            }
        }
    }

}

bool Graph::updateNextRoadForCar(Car *c, int crossId, int idealRoad)
{

    if (idealRoad==-1) {
        c->nextRoad=c->onRoad;
        c->direction=FORWARD;
        return true;
    }

    if (c->onRoad!=c->nextRoad) {
        return false;
    }

    vector<Edge*> options;
    int originDist = lookAtDist(crossId,c->to);
    vector<int> neighbors = id2cross[crossId].neighbors;
    for (size_t i=0; i<neighbors.size(); ++i) {
        if (neighbors[i]==-1 || neighbors[i]==c->onRoad || neighbors[i]==idealRoad) continue;
        int roId=neighbors[i];
        Edge* r = (id2road[roId].first.from==crossId)? &id2road[roId].first : &id2road[roId].second;
        if (r->channels>0) {
            int newDist=lookAtDist(r->to,c->to);
            if (newDist<=originDist) {
                options.push_back(r);
            }
        }
    }
    if (options.empty()) {
        c->nextRoad=idealRoad;
        c->path.push_back(idealRoad);
        int in, out;
        for (size_t i=0; i<neighbors.size(); ++i) {
            if (neighbors[i]==c->onRoad) in=i;
            if (neighbors[i]==c->nextRoad) out=i;
        }
        int res=out-in; // update direction
             if (res==1 || res==-3) c->direction=LEFT;
        else if (res==2 || res==-2) c->direction=FORWARD;
        else if (res==3 || res==-1) c->direction=RIGHT;
        return false;
    }
    else {
        vector<double> score;
        for (size_t i=0; i<options.size(); ++i) {
            Edge *r=options[i];
            double occu = r->occupancyRate;
            score.push_back(occu);
        }
        int minIdx = min_element(score.begin(),score.end())-score.begin();
        double minOccu = score[minIdx];
        Edge* r = (id2road[idealRoad].first.from==crossId)? &id2road[idealRoad].first : &id2road[idealRoad].second;
        if (r->occupancyRate < minOccu + threshold) {
            c->nextRoad=idealRoad;
            c->path.push_back(idealRoad);
        }
        else {
            c->nextRoad=options[minIdx]->id;
            c->path.push_back(options[minIdx]->id);
        }
        int in, out;
        for (size_t i=0; i<neighbors.size(); ++i) {
            if (neighbors[i]==c->onRoad) in=i;
            if (neighbors[i]==c->nextRoad) out=i;
        }
        int res=out-in; // update direction
             if (res==1 || res==-3) c->direction=LEFT;
        else if (res==2 || res==-2) c->direction=FORWARD;
        else if (res==3 || res==-1) c->direction=RIGHT;
        return false;
    }




}

// for debug
int Graph::printUnfinished()
{
    int countWaiting=0;
    for (auto &car:id2car) {
        Car *c=&car.second;
        if (c->status==WAITING) {
            countWaiting++;
//            cout<<"id:"<<c->id<<endl;
//            cout<<"onroad:"<<c->onRoad<<endl;
//            cout<<"nextroad:"<<c->nextRoad<<endl;
//            cout<<"dis2Cross:"<<c->dis2Cross<<endl;
//            cout<<"dis2Prev:"<<c->dis2Prev<<endl;
//            cout<<"direction:"<<c->direction<<endl;
//            cout<<"maxSpeed:"<<c->maxSpeed<<endl;
//            cout<<"speed:"<<c->speed<<endl;
//            cout<<"from:"<<c->from<<endl;
//            cout<<"to:"<<c->to<<endl;
//            cout<<"status:"<<c->status<<endl;
//            cout<<endl;
        }
    }
    return countWaiting;
}

void Graph::printTotalOccupy()
{
    for (auto &edges:id2road) {
        Edge *e1=&edges.second.first;
        Edge *e2=&edges.second.second;
        totalOccupy += e1->length*e1->channels;
        totalOccupy += e2->length*e2->channels;
    }
    cout << "total occupy num: " << totalOccupy <<endl;
}

void Graph::adjustPlanningTime()
{

    for (auto &car:id2car) {
        Car *c=&car.second;
        planningTime2Cars[c->planTime].push_back(c->id);
        from2to[c->from].emplace(c->to);
        mainDirCount[c->mainDir]++;
    }

    map<int, vector<int>>::iterator it;
    it=planningTime2Cars.begin();
    while (it!=planningTime2Cars.end()) {
        for (size_t i=0; i<it->second.size(); ++i) {
            int carId=it->second[i];
            Car *c=&id2car[carId];

            if (c->mainDir==10 || c->mainDir==32) {
                c->planTime=100;
            }
            else if (c->mainDir==4 || c->mainDir==7) {
                c->planTime=250;
            }
            else if (c->mainDir==12 || c->mainDir==30) {
                c->planTime=350;
            }

        }
        it++;
    }



}

void Graph::printFinishInfo()
{
    int priFirst=INT_MAX;
    int priLast=INT_MIN;
    int priStep=0;
    int totalPriStep=0;
    int step=0;
    int totalStep=0;

    for (auto &car:id2car) {
        Car *c=&car.second;
        assert(c->status==FINISH);
        if (c->priority) {
            priFirst=min(priFirst, c->originalPlanTime);
            priLast=max(priLast, c->finishTime);
            totalPriStep += c->actualSteps;
            totalStep += c->actualSteps;
            step = max(step, c->finishTime);
        }
        else {
            totalStep += c->actualSteps;
            step = max(step, c->finishTime);
        }
    }

    priStep = priLast-priFirst;
    cout<< "STEP: "<<step<<endl;
    cout<< "TOTAL STEP: "<<totalStep<<endl;
    cout<< "PRIORITY STEP: "<<priStep<<endl;
    cout<< "TOTAL PRIORITY STEP: "<<totalPriStep<<endl;
}

vector<int> Graph::getCarsOnRoad()
{
    vector<int> onRoad;
    for (auto &car:id2car) {
        Car *c=&car.second;
        if (c->status!=FINISH && c->status!=UNDEPLOYED) {
            onRoad.push_back(c->id);
        }
    }
    return onRoad;
}

void Graph::printPresstCars()
{
    int num=0;
    for (auto &car:id2car) {
        Car *c=&car.second;
        if (c->preset) {
            if (c->originalPlanTime!=c->planTime) {
                num++;
                cout<<"preset Car Id: "<<c->id<<" origin: "<<c->originalPlanTime<<" actual: "<<c->planTime<<endl;
            }
        }
    }
    cout<<"total : "<<num<<endl;
}

