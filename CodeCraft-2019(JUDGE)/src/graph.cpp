#include "../include/graph.h"
#include<algorithm>
#include<limits>
#include<unordered_map>

Graph::Graph()
{
    numEdges=0;
    numVetices=0;
    numCars=0;
    totalOccupy=0.0;
//    mainDirCount.resize(8,0);
    firstTime=true;

    occupancyLimit=1.0;
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
//    // fill adj list
//    for (auto &edges:id2road) {
//        Edge *e1=&edges.second.first;
//        Edge *e2=&edges.second.second;
//        adj[e1->from].push_back(make_pair(e1->to,e1->length));
//        adjBackup[e1->from].push_back(make_pair(e1->to,e1->length));
//        if (e2->channels!=0) {
//            adj[e2->from].push_back(make_pair(e2->to,e2->length));
//            adjBackup[e2->from].push_back(make_pair(e2->to,e2->length));
//        }
//    }

//    // generate path for non-preseted cars
//    for (auto &car:id2car) {
//        Car *c=&car.second;
//        if (c->preset==false) {
//            vector<int> result=generatePath(c->from,c->to);
//            c->path=result;
//        }
//        getOptStepsForCar(c);
//    }

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
    {
        int i=0;
        for (auto &car:id2car) {
            Car *c=&car.second;
            cout << "path from "<<c->from<<" to "<<c->to<<" :"<<endl;
            for (size_t j=0; j<c->path.size(); ++j) {
                cout<<"->"<<c->path[j];
            }
            cout<<endl;
            ++i;
            if (i>=10) break;
        }
    }


}

void Graph::loadGraphDirect()
{
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

    int count=0;
    for (auto &car:id2car) {
        Car *c=&car.second;
        // for test
//        if (c->speedLevel==FAST) {
//            continue;
//        }
        // end
        vector<int> candidates=c->getTransitionPoint(numVetices);
        int candidate1=candidates[0], candidate2=candidates[1];
        bool connect1=directConnected(c->from,candidate1);
        bool connect2=directConnected(candidate1,c->to);
        vector<int> res;
        if (connect1&&connect2) {
            count++;
            vector<int> res1=generatePath(c->from,candidate1);
            vector<int> res2=generatePath(candidate1,c->to);
            res.insert(res.end(),res2.begin(),res2.end());
            res.insert(res.end(),res1.begin(),res1.end());
        }
        else {
            if (candidate2!=0) {
                bool connect3=directConnected(c->from, candidate2);
                bool connect4=directConnected(candidate2, c->to);
                if (connect3&&connect4) {
                    count++;
                    vector<int> res3=generatePath(c->from,candidate2);
                    vector<int> res4=generatePath(candidate2,c->to);
                    res.insert(res.end(),res4.begin(),res4.end());
                    res.insert(res.end(),res3.begin(),res3.end());
                }
            }
        }
        if (res.empty()) {
            continue;
        }

        int fromId=c->from;
        for (int j=res.size()-1; j>=0; --j) {
            int toId=res[j];
            int roadId=cross2road[pair<int,int>(fromId,toId)];
            id2roadCount[pair<int,int>(fromId,toId)]++;
            c->path.push_back(roadId);
            vector<int> neighbor=id2cross[fromId].neighbors;
            for (size_t idx=0; idx<neighbor.size(); idx++) {
                if (roadId==neighbor[idx]) {
                    c->pathDirCount[idx]++;
                    break;
                }
            }
            fromId=toId;
        }
        getOptStepsForCar(c);

    }
    cout<<"num of direct path car: " <<count<< endl;

    double addWeight=0.02;
    for (auto &road : id2roadCount) {
        int fromId=road.first.first;
        int toId=road.first.second;
        int num=road.second;
        int n=adj[fromId].size();
        for (int i=0; i<n; ++i) {
            if (toId==adj[fromId][i].first) {
                adj[fromId][i].second += num*addWeight;
                break;
            }
        }
    }

    for (auto &car:id2car) {
        Car *c=&car.second;
        if (c->path.empty()) {
            vector<int> res=generatePath(c->from, c->to);
            int fromId=c->from;
            for (int j=res.size()-1; j>=0; --j) {
                int toId=res[j];
                int roadId=cross2road[pair<int,int>(fromId,toId)];
                id2roadCount[pair<int,int>(fromId,toId)]++;
                c->path.push_back(roadId);

                vector<pair<int, int>> *neighb=&adj[fromId];
                int n = neighb->size();
                for (int i=0; i<n; ++i) {
                    if (neighb->at(i).first==toId) {
                        neighb->at(i).second += addWeight;
                    }
                    break;
                }

                vector<int> neighbor=id2cross[fromId].neighbors;
                for (size_t idx=0; idx<neighbor.size(); idx++) {
                    if (roadId==neighbor[idx]) {
                        c->pathDirCount[idx]++;
                        break;
                    }
                }
                fromId=toId;
            }

            getOptStepsForCar(c);
        }
    }

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
    return result;
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
        cout<<"LOOP: "<<loops<<endl;
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
                    bool isFinal = c->updateNextRoad(cro->neighbors);
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
                                        c->finishTime = time;
                                        c->actualSteps = time - c->originalPlanTime;
                                        r->updateOccupy(c,REMOVE,c->inChannel);
                                    }
                                    else {
                                        Edge* strEdge = (id2road[straight].first.from==cro->id) ? &id2road[straight].first : &id2road[straight].second;
                                        if (strEdge->channels==0) {
                                            c->moveToEnd();
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
                        Edge *nextEdge=(id2road[c->nextRoad].first.from==cro->id) ? &id2road[c->nextRoad].first : &id2road[c->nextRoad].second;
                        Car *prevCar=r->findPrevCar(c,id2car);
                        c->updateCurrentCar(prevCar,r->speed);
                        vector<int> crossCarList=createCarSequeue(cro->id);
                        assert(!crossCarList.empty());

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
void Graph::scheduleNewCars(int time)
{

    double roadLimit=0.8;
    int countForCheck=0;
    int maxNum=64;
    int curNum=0;
    int initialCarNumEachRoad=2;
    double undeployRateLimit=0.3;


    if (time<=5) {
        maxNum=100;
        map<pair<int,int>, int> initialCount;
        map<pair<int,int>, int>::iterator it;
        for (it=cross2road.begin();it!=cross2road.end();it++) {
            initialCount[it->first]=initialCarNumEachRoad;
        }

        for (auto &car:id2car) {
            if (curNum>=maxNum) break;
            Car* c = &car.second;
            if (c->planTime<=time && c->status==UNDEPLOYED) {
                int roadNum=c->path[0];
                Edge *r=(id2road[roadNum].first.from==c->from) ? &id2road[roadNum].first : &id2road[roadNum].second;
                if (r->channels==0) {
                    cout<<"ERROR: the edge of road "<<roadNum<<" does not exist"<<endl;
                }

                int carSpeed=c->speed;
                int roadSpeed=r->speed;
                if (carSpeed==roadSpeed && initialCount[make_pair(r->from,r->to)]>0) {
                    if (r->occupancyRate<roadLimit) {
                        bool res=r->addNewCar(c);
                        if (res) {
                            c->planTime=time;
                            initialCount[make_pair(r->from,r->to)]--;
                            curNum++;
                            cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                        }
                    }
                }

            }
        }
        undeployRate=getUndeployRate();
    }

    else {

        if (undeployRate<undeployRateLimit) {

                // update weight
                updateAdjWeight();
                // change path for undeployed cars
                for (auto &car:id2car) {
                    Car *c=&car.second;
                    if (c->status==UNDEPLOYED) {
                        c->path.clear();
                        vector<int> res=generatePath(c->from, c->to);
                        int fromId=c->from;
                        for (int j=res.size()-1; j>=0; --j) {
                            int toId=res[j];
                            int roadId=cross2road[pair<int,int>(fromId,toId)];
                            c->path.push_back(roadId);
                            fromId=toId;
                        }
                        getOptStepsForCar(c);
                    }
                }
                occupancyLimit=0.12;
                firstTime=false;



            vector<int> res=getSlowCars();
            if (!res.empty()) {
                cout<<"HERE HERE!!! "<<res.size()<<endl;
                for (size_t i=0; i<res.size(); ++i) {
                    int carId=res[i];
                    Car *c=&id2car[carId];
                    c->planTime=time;
                    int roadNum=c->path[0];
                    Edge *r=(id2road[roadNum].first.from==c->from) ? &id2road[roadNum].first : &id2road[roadNum].second;
                    if (r->channels==0) {
                        cout<<"ERROR: the edge of road "<<roadNum<<" does not exist"<<endl;
                    }
                    bool res=r->addNewCar(c);
                    if (res) {
                        c->planTime=time;
                        curNum++;
                        cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                    }

                }
            }

            for (auto &car:id2car) {
                if (curNum>=maxNum) break;
                if (countForCheck%100==0) {
                    countForCheck=0;
                    totalOccupancyRate=getTotalOccupancyRate();
                    undeployRate=getUndeployRate();
                }
                countForCheck++;
                if (totalOccupancyRate >= occupancyLimit) {
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
                                curNum++;
                                cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                            }
                        }
                    }
                }
            }

        }

        else {
//            for (auto &car:id2car) {
//                if (curNum>=maxNum) break;
//                if (countForCheck%100==0) {
//                    countForCheck=0;
//                    totalOccupancyRate=getTotalOccupancyRate();
//                    undeployRate=getUndeployRate();
//                }
//                countForCheck++;
//                if (totalOccupancyRate >= occupancyLimit) {
//                    break;
//                }
//                else {
//                    Car* c = &car.second;
//                    if (c->planTime<=time && c->status==UNDEPLOYED) {
//                        int roadNum=c->path[0];
//                        Edge *r=(id2road[roadNum].first.from==c->from) ? &id2road[roadNum].first : &id2road[roadNum].second;
//                        if (r->channels==0) {
//                            cout<<"ERROR: the edge of road "<<roadNum<<" does not exist"<<endl;
//                        }

//                        if (r->occupancyRate<roadLimit) {
//                            bool res=r->addNewCar(c);
//                            if (res) {
//                                c->planTime=time;
//                                curNum++;
//                                cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
//                            }
//                        }
//                    }
//                }
//            }


            //choose car for each road
            map<pair<int,int>, double> road2OccupancyRate;
            map<pair<int,int>, int>::iterator it;
            for (it=cross2road.begin(); it!=cross2road.end(); it++) {
                int roadId=it->second;
                int fromId=it->first.first;
                Edge *r=(id2road[roadId].first.from==fromId) ? &id2road[roadId].first : &id2road[roadId].second;
                road2OccupancyRate[it->first]=r->getOccupancyRate();
            }

            vector<PAIR> roadList;
            copy(road2OccupancyRate.begin(),road2OccupancyRate.end(),back_inserter<vector<PAIR>>(roadList));
            sort(roadList.begin(), roadList.end(),
                 Compare());

            for (auto &pair : roadList) {
                if (curNum>=maxNum) break;
                if (countForCheck%100==0) {
                    countForCheck=0;
                    totalOccupancyRate=getTotalOccupancyRate();
                    undeployRate=getUndeployRate();
                }
                countForCheck++;
                if (totalOccupancyRate >= occupancyLimit) {
                    break;
                }

                int fromId=pair.first.first;
                double occupancyRate=pair.second;
//                cout<<"occupancyRate: "<<occupancyRate<<endl;


                int roadId=cross2road[pair.first];
                Edge *r=(id2road[roadId].first.from==fromId) ? &id2road[roadId].first : &id2road[roadId].second;
                vector<int> cars=getCarsForRoad(r);
                int roadSpeed = r->speed;
                vector<vector<int>> occupy=r->occupy;
                int limitSpeed=0; // vacancy in order
                for (int j=0; j<occupy.size(); ++j) {
                    for (size_t i=occupy[0].size()-roadSpeed; i<occupy[0].size(); ++i) {
                        if (occupy[j][i]==0) {
                            limitSpeed=occupy[0].size()-i;
                            break;
                        }
                    }
                }

                int newCarNum=1;
                if (occupancyRate<0.3) newCarNum=3;

                if (!cars.empty() && limitSpeed!=0) {

                    while (limitSpeed>0 && newCarNum>0) {
                        int maxSpeed=0;
                        for (size_t i=0; i<cars.size(); ++i) {
                            int carId=cars[i];
                            Car *c=&id2car[carId];
                            int carSpeed=c->speed;
                            maxSpeed=max(carSpeed,maxSpeed);
                            if (carSpeed!=limitSpeed) continue;
                            bool res=r->addNewCar(c);
                            if (res) {
                                c->planTime=time;
                                curNum++;
                                cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                newCarNum--;
                            }
                        }
                        if (maxSpeed<limitSpeed) limitSpeed=maxSpeed;
                        else limitSpeed--;
                    }

                }

            }


        }


    }



}

void Graph::scheduleNewCars(int time, bool priorOn)
{
    double roadLimit=1.0;
    int countForCheck=0;
    int maxNum=6400;
    int curNum=0;

    if (priorOn) {
        for (auto &edges:id2road) {
            if (curNum>=maxNum) break;
            if (countForCheck%100==0) {
                countForCheck=0;
                totalOccupancyRate=getTotalOccupancyRate();
            }
            if (totalOccupancyRate >= occupancyLimit) {
                break;
            }

            Edge *e1=&edges.second.first;
            Edge *e2=&edges.second.second;
            if (!e1->priorCarList.empty()) {
                for (size_t i=0; i<e1->priorCarList.size(); ++i) {
                    countForCheck++;
                    int carId=e1->priorCarList[i];
                    Car* c=&id2car[carId];

                    if (c->planTime>time) continue;
                    if (e1->occupancyRate<roadLimit) {
                        bool res=e1->addNewPriorCar(c, id2car);
                        if (res) {
//                            c->planTime=time;
                            cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                            e1->updatePriorList(id2car);
                            curNum++;
                            i--;
                        }
                    }

                }
            }

            if (e2->channels!=0) {
                if (!e2->priorCarList.empty()) {
                    for (size_t i=0; i<e2->priorCarList.size(); ++i) {
                        countForCheck++;
                        int carId=e2->priorCarList[i];
                        Car* c=&id2car[carId];

                        if (c->planTime>time) continue;
                        if (e2->occupancyRate<roadLimit) {
                            bool res=e2->addNewPriorCar(c, id2car);
                            if (res) {
//                                c->planTime=time;
                                cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                e2->updatePriorList(id2car);
                                curNum++;
                                i--;
                            }
                        }

                    }
                }
            }
        }


    }
    else {
        for (auto &edges:id2road) {
            if (curNum>=maxNum) break;
            if (countForCheck%100==0) {
                countForCheck=0;
                totalOccupancyRate=getTotalOccupancyRate();
            }
            if (totalOccupancyRate >= occupancyLimit) {
                break;
            }
            Edge *e1=&edges.second.first;
            Edge *e2=&edges.second.second;
            if (!e1->priorCarList.empty()) {
                for (size_t i=0; i<e1->priorCarList.size(); ++i) {
                    countForCheck++;
                    int carId=e1->priorCarList[i];
                    Car* c=&id2car[carId];
                    if (c->planTime>time) continue;
                    if (e1->occupancyRate<roadLimit) {
                        bool res=e1->addNewPriorCar(c, id2car);
                        if (res) {
//                            c->planTime=time;
                            cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                            e1->updatePriorList(id2car);
                            curNum++;
                            i--;
                        }
                    }

                }
            }

            //add non-prior cars
            if (!e1->carList.empty()) {
                for (size_t i=0; i<e1->carList.size(); ++i) {
                    countForCheck++;
                    int carId=e1->carList[i];
                    Car* c=&id2car[carId];
                    if (c->planTime>time) continue;
                    if (e1->occupancyRate<roadLimit) {
                        bool res=e1->addNewCar(c);
                        if (res) {
//                            c->planTime=time;
                            cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                            e1->updateCarList(id2car);
                            curNum++;
                            i--;
                        }
                    }

                }
            }

            if (e2->channels!=0) {
                if (!e2->priorCarList.empty()) {
                    for (size_t i=0; i<e2->priorCarList.size(); ++i) {
                        countForCheck++;
                        int carId=e2->priorCarList[i];
                        Car* c=&id2car[carId];
                        if (c->planTime>time) continue;
                        if (e2->occupancyRate<roadLimit) {
                            bool res=e2->addNewPriorCar(c, id2car);
                            if (res) {
//                                c->planTime=time;
                                cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                e2->updatePriorList(id2car);
                                curNum++;
                                i--;
                            }
                        }

                    }
                }

                //add non-prior cars
                if (!e2->carList.empty()) {
                    for (size_t i=0; i<e2->carList.size(); ++i) {
                        countForCheck++;
                        int carId=e2->carList[i];
                        Car* c=&id2car[carId];
                        if (c->planTime>time) continue;
                        if (e2->occupancyRate<roadLimit) {
                            bool res=e2->addNewCar(c);
                            if (res) {
//                                c->planTime=time;
                                cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                                e2->updateCarList(id2car);
                                curNum++;
                                i--;
                            }
                        }

                    }
                }
            }


        }



    }
}

    //original version
//    void Graph::scheduleNewCars(int time)
//    {
//        for (auto &car:id2car) {
//            Car* c = &car.second;
//            if (c->planTime<=time && c->status==UNDEPLOYED) {
//                int roadNum=c->path[0];
//                Edge *r=(id2road[roadNum].first.from==c->from) ? &id2road[roadNum].first : &id2road[roadNum].second;
//                if (r->channels==0) {
//                    cout<<"ERROR: the edge of road "<<roadNum<<" does not exist"<<endl;
//                }
//                bool res=r->addNewCar(c);
//                if (res) {
//                    cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
//                }
//            }
//        }
//    }


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

    // clear waiting queue
//        for (auto &cross:id2cross) {
//            Vertex *v=&cross.second;
//            if (!v->waitingQueue.empty()) {
//                cout<<"EEOR: cross id: "<<v->id<<" still has road in waiting queue"<<endl;
//            }
//            while(!v->waitingQueue.empty()) {
//                v->waitingQueue.pop();
//            }
//        }

    cout << "checking full roads..."<<endl;
    for (auto &edges:id2road) {
        Edge *e1=&edges.second.first;
        Edge *e2=&edges.second.second;
        if (e1->channels!=0) {
            if (e1->isFull()) cout<<e1->id<<"from "<<e1->from<<" to "<<e1->to<<endl;
        }
        if (e2->channels!=0) {
            if (e2->isFull()) cout<<e2->id<<"from "<<e2->from<<" to "<<e2->to<<endl;
        }

        double r1=e1->getOccupancyRate();
        e1->time2occupancy.push_back(r1);
        double r2=e2->getOccupancyRate();
        e2->time2occupancy.push_back(r2);

    }

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
    Car::count=0;
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

double Graph::getTotalOccupancyRate()
{
    int carNum=0;
    map<int, pair<Edge, Edge>>::iterator it;
    it=id2road.begin();
    while (it!=id2road.end()) {
        Edge *e1 = &it->second.first;
        Edge *e2 = &it->second.second;
        carNum += e1->getNumCars();
        if (e2->channels!=0) {
            carNum += e2->getNumCars();
        }
        it++;
    }
    totalOccupancyRate=double(carNum)/double(totalOccupy);
    return totalOccupancyRate;
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
            c->updateNextRoad(v->neighbors);
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
    road->updatePriorList(id2car);
    if (!road->priorCarList.empty()) {
        for (size_t i=0; i<road->priorCarList.size(); ++i) {
            int carId=road->priorCarList[i];
            Car* c=&id2car[carId];

            if (c->planTime > time) continue;
            bool res=road->addNewPriorCar(c, id2car);
            if (res) {
                cout<<"Car No."<<c->id<<" get on road at step "<<time<<endl;
                road->updatePriorList(id2car);
                i--;
            }
        }
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
//    for (auto &car:id2car) {
//        Car *c=&car.second;
//        if (c->status==FINISH) {
//            cout<<"id: "<<c->id;
//            cout<<" optSteps: "<<c->optSteps;
//            cout<<" actualSteps: "<<c->actualSteps;
//            cout<<" planTime: "<<c->planTime<<endl;
//            cout<<endl;
//            id2time[c->id]=c->actualSteps-c->optSteps;
//        }
//    }
//    cout<<"**********************"<<endl;

//    copy(id2time.begin(),id2time.end(),back_inserter<vector<pair<int,int>>>(waitingTime));
//    sort(waitingTime.begin(),waitingTime.end(),
//         [](const pair<int,int>& l, const pair<int,int>& r){
//            if (l.second!=r.second)
//                return l.second>r.second;
//            else return l.first>r.first;
//        });
//    for (int i=0; i<1000; ++i) {
//        int carId=waitingTime[i].first;
//        int extraTime=waitingTime[i].second;
//        Car *c=&id2car[carId];
//        cout<<"id: "<<carId;
//        cout<<" planTime: "<<c->planTime;
//        cout<<" speed: "<<c->speed;
//        cout<<" waitingTime: "<<extraTime<<endl;
//        cout<<endl;
//    }

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

