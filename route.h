#ifndef __ROUTE_H__
#define __ROUTE_H__

#include "lib_record.h"
#include <stdio.h>
#include <iostream>
#include <vector>
#include <ext/hash_map>

using namespace std;

#define INFINITE 1000000
#define ID 0
#define SOURCE 1
#define DEST 2
#define COST 3
#define MAX_NODE_NUM 600

class neighbor{
public:
    int vertex;       //ID of the neighbor
    int LinkID;       //ID of that Link
    int cost;         //cost of that Link

    neighbor(){}
    neighbor(int v, int l, int c):vertex(v),LinkID(l),cost(c){}
};

class path{
public:
    int src;
    int dest;
    int cost;
    vector<int> nodes;       //vertex path
    vector<int> edges;       //edge path

    path(){}
    path(int s, int d, int c):src(s),dest(d),cost(c){}

};

void search_route(char *graph[5000], int edge_num, char *condition);
void resolve(char * e, int* result);
void create_matrix(char *topo[5000], int edge_num, int &node_num, vector< vector<neighbor> > &m);
void resolve_demand(char *e, int &source, int &dest, vector<int> &Vs);
void dijkstra(int source, int noList[], int noListSize);
path mergePath(path p1, path p2);

#endif
