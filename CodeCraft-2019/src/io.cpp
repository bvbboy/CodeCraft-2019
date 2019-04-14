#include "../include/lib_io.h"
#include "../include/edge.h"
#include "../include/graph.h"
#include "../include/car.h"
#include<fstream>
#include<iostream>
#include<sstream>
using namespace std;

int read_crossfile(string crossfile, Graph &g)
{
    int count=0;
    string line;
    ifstream in(crossfile);
    if (!in) {
        cout<<"Cannot open crossfile!"<<endl;
        return 0;
    }
    while (getline(in, line)) {
        if (line.front()=='#') continue;
        if (line.front()=='(' && (line.back()==')'||line.back()=='\r')) {
            ++count;
            int n=line.size();
            if (line.back()==')') line=line.substr(1,n-2);
            else if (line.back()=='\r') line=line.substr(1,n-3);
            stringstream sstr(line);
            string item;
            int idx=0;
            int id=0, road0, road1, road2, road3;
            while (getline(sstr, item, ',')) {
                item.erase(0,item.find_first_not_of(" "));
                item.erase(item.find_last_not_of(" ")+1);
                switch (idx) {
                case 0:
                    id=stoi(item);
                    break;
                case 1:
                    road0=stoi(item);
                    break;
                case 2:
                    road1=stoi(item);
                    break;
                case 3:
                    road2=stoi(item);
                    break;
                case 4:
                    road3=stoi(item);
                    break;
                default:
                    break;
                }
                ++idx;
//                cout<<item<<" ";
            }
            Vertex newV(id);
            newV.addNeighbor(road0,road1,road2,road3);
            g.addVertex(newV);
//            cout<<endl;
        }
    }
    return count;
}

int read_roadfile(string roadfile, Graph &g)
{
    int count=0;
    string line;
    ifstream in(roadfile);
    if (!in) {
        cout<<"Cannot open roadfile!"<<endl;
        return 0;
    }
    while (getline(in, line)) {
        if (line.front()=='#') continue;
        if (line.front()=='(' && (line.back()==')'||line.back()=='\r')) {
            ++count;
            int n=line.size();
            if (line.back()==')') line=line.substr(1,n-2);
            else if (line.back()=='\r') line=line.substr(1,n-3);
            stringstream sstr(line);
            string item;
            int idx=0;
            int id,length,speed,channels,from,to;
            bool isDup;
            while (getline(sstr, item, ',')) {
                item.erase(0,item.find_first_not_of(" "));
                item.erase(item.find_last_not_of(" ")+1);
                switch (idx) {
                case 0:
                    id=stoi(item);
                    break;
                case 1:
                    length=stoi(item);
                    break;
                case 2:
                    speed=stoi(item);
                    break;
                case 3:
                    channels=stoi(item);
                    break;
                case 4:
                    from=stoi(item);
                    break;
                case 5:
                    to=stoi(item);
                    break;
                case 6:
                    isDup=stoi(item);
                    break;
                default:
                    break;
                }
                ++idx;
//                cout<<item<<" ";
            }
            Edge newEdge(id,from,to,length,speed,channels,isDup);
            g.addEdge(newEdge);
//            cout<<endl;
        }
    }
    return count;
}

int read_carfile(string carfile, Graph &g)
{
    int count=0;
    string line;
    ifstream in(carfile);
    if (!in) {
        cout<<"Cannot open carfile!"<<endl;
        return 0;
    }
    while (getline(in, line)) {
        if (line.front()=='#') continue;
        if (line.front()=='(' && (line.back()==')'||line.back()=='\r')) {
            ++count;
            int n=line.size();
            if (line.back()==')') line=line.substr(1,n-2);
            else if (line.back()=='\r') line=line.substr(1,n-3);
            stringstream sstr(line);
            string item;
            int idx=0;
            int id,from,to,speed,planTime,prior,preset;
            while (getline(sstr, item, ',')) {
                item.erase(0,item.find_first_not_of(" "));
                item.erase(item.find_last_not_of(" ")+1);
                switch (idx) {
                case 0:
                    id=stoi(item);
                    break;
                case 1:
                    from=stoi(item);
                    break;
                case 2:
                    to=stoi(item);
                    break;
                case 3:
                    speed=stoi(item);
                    break;
                case 4:
                    planTime=stoi(item);
                    break;
                case 5:
                    prior=stoi(item);
                    break;
                case 6:
                    preset=stoi(item);
                    break;
                default:
                    break;
                }
                ++idx;
//                cout<<item<<" ";
            }
            Car newC(id,from,to,speed,planTime,prior,preset);
            g.addCar(newC);
//            cout<<endl;
        }
    }
    return count;
}

int read_presetcarfile(string presetcarfile, Graph &g) {
    int count = 0;
    string line;
    ifstream in(presetcarfile);
    if (!in) {
        cout<<"Cannot open presetfile!"<<endl;
        return 0;
    }
    while(getline(in, line)) {
        if (line.front() == '#') continue;
        if (line.front() == '(' && (line.back() == ')' || line.back() == '\r')) {
            ++count;
            int n = line.size();
            if (line.back() == ')') line = line.substr(1, n - 2);
            else if (line.back() == '\r') line = line.substr(1, n - 3);
            stringstream sstr(line);
            string item;
            int idx = 0;
            int carId;
            int t;
            while (getline(sstr, item, ',')) {
                item.erase(0, item.find_first_not_of(" "));
                item.erase(item.find_last_not_of(" ") + 1);
                switch (idx) {
                    case 0:
                        carId = stoi(item);
                        g.numPresetCars++;
                        break;
                    case 1:
                        t=stoi(item);
                        g.id2car[carId].planTime = t;
                        g.id2car[carId].originalPlanTime = t;
                        break;
                    default:
                        g.id2car[carId].path.push_back(stoi(item));
                        break;
                }
                ++idx;
            }

        }
    }
    return count;
}

int read_answerfile(string answerfile, Graph &g) {
    int count=0;
    string line;
    ifstream in(answerfile);
    if (!in) {
        cout<<"Cannot open answerfile!" <<endl;
        return 0;
    }
    while (getline(in,line)) {
        count++;
        int n=line.size();
        if(line.front()=='(' && line.back()==')') {
            line = line.substr(1,n-2);
            string carId=line.substr(0,line.find_first_of(','));
            int id =stoi(carId);
            Car *car=&g.id2car[id];
            car->path.clear();
            line = line.substr(line.find_first_of(',')+1);
            string planT=line.substr(0,line.find_first_of(','));
            int planTime=stoi(planT);
            car->planTime=planTime;
            line = line.substr(line.find_first_of(',')+1);
            stringstream sstr(line);
            string item;
            while (getline(sstr, item, ',')) {
                int path=stoi(item);
                car->path.push_back(path);
            }
        }
    }
    return count;


}


void write_answerfile(string answerfile, Graph &g) {

    ofstream out(answerfile);
    for (auto &car:g.id2car) {
        Car *c=&car.second;
        if (c->preset) continue;
        string line;
        line += "(";
        line += to_string(c->id);
        line += ",";
        line += to_string(c->planTime);
        for (size_t i=0; i<c->path.size(); ++i) {
            line += ",";
            line += to_string(c->path[i]);
        }
        line += ")\n";
        out << line;
    }
    out.close();
}
