#ifndef SOLUTION_H
#define SOLUTION_H

#include "graph.h"

using namespace std;

class Solution
{
public:
    Solution();

    int processSchdule(Graph &graph, int steps);

    void adjustSomething(Graph &graph);

public:
    int steps; //count time


};

#endif // SOLUTION_H
