#ifndef LIB_IO_H
#define LIB_IO_H

#include<string>
#include "graph.h"
using namespace std;


extern int read_crossfile(string crossfile, Graph &g);
extern int read_roadfile(string roadfile, Graph &g);
extern int read_carfile(string carfile, Graph &g);
extern int read_presetcarfile(string presetcarfile, Graph &g);

extern int read_answerfile(string answerfile, Graph &g);

extern void write_answerfile(string answerfile, Graph &g);

#endif // LIB_IO_H
