#include "solution.h"

Solution::Solution()
{
    steps=0;
}

int Solution::processSchdule(Graph &graph, int steps)
{
    while(!graph.checkStatus()&&steps<=5000) {
        ++steps;
        cout<<"in steps: "<<steps<<endl;

        graph.processStatus();
        graph.scheduleNewCars(steps,true);
        bool deadLock=graph.schedule(steps);
        if (deadLock) {
            cout<<"deadlock at step "<<steps<<endl;
            return 0;
        }
        graph.scheduleNewCars(steps,false);
        graph.resetStatus();
        cout<<"finish num: " <<Car::countFinish<<endl;
    }
    graph.printFinishInfo();
    graph.printPresstCars();

    return steps;
}

void Solution::adjustSomething(Graph &graph)
{
    int fraction=0.1;
    int width=10;
    int num=graph.waitingTime.size();
    for (int i=0; i<num*fraction; ++i) {
        int carId = graph.waitingTime[i].first;
        Car *c=&graph.id2car[carId];
        int roadId=c->path[0];
        Edge *e=(graph.id2road[roadId].first.from==c->from) ? &graph.id2road[roadId].first : &graph.id2road[roadId].second;
        vector<double> occ = e->time2occupancy;
        int begin=c->planTime-width;
        int end=c->planTime+width;
        begin=max(2,begin);
        end=min(end,int(0.9*occ.size()));
        double lowestOcc=graph.waitingTime[i].second;
        int planTime=c->planTime;
        for (int i=begin; i<end; ++i) {
            if (occ[i]<lowestOcc) {
                lowestOcc=occ[i];
                planTime=i;
            }
        }
        if (planTime==graph.waitingTime[i].second) {
            c->planTime=begin;
        }
        else {
            c->planTime=i;
        }
    }

    // reset some states

    graph.reset();

}
