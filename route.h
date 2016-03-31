#ifndef __ROUTE_H__
#define __ROUTE_H__

#include "lib_record.h"
#include <stdio.h>
#include <iostream>
#include <vector>
#include <ext/hash_map>
#include <algorithm>

using namespace std;

#define INFINITE 1000000
#define ID 0
#define SOURCE 1
#define DEST 2
#define COST 3
#define MAX_NODE_NUM 600
#define RETAIN_LEVEL 2

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
    int middle;
    int passNodes;
    vector<int> nodes;       //vertex path
    vector<int> edges;       //edge path

    path(){}
    path(int s, int d, int c):src(s),dest(d),cost(c),middle(-1),passNodes(0){}

    bool isLoopless(){
        __gnu_cxx::hash_map<int, int> m;
        for(int i=0;i<nodes.size();i++){
            if(m.find(nodes[i]) != m.end()) return false;
            else m[nodes[i]] = 0;
        }
        return true;
    }

    void printPath(){
        cout<<"from "<<src<<" to "<<dest<<" cost: "<<cost<<endl;
        for(int k=0;k<nodes.size()-1;k++)
            cout<<nodes[k]<<"->";
        cout<<nodes[nodes.size()-1]<<endl;
        for(int k=0;k<edges.size()-1;k++)
            cout<<edges[k]<<"|";
        cout<<edges[edges.size()-1]<<endl;
    }

    void count(__gnu_cxx::hash_map<int, int> node_vs){
        for(int i=1;i<nodes.size();i++){
            if(node_vs.find(nodes[i]) != node_vs.end()) passNodes++;
        }
    }
};

void search_route(char *graph[5000], int edge_num, char *condition);
void resolve(char * e, int* result);
void create_matrix(char *topo[5000], int edge_num, int &node_num, vector< vector<neighbor> > &m);
void resolve_demand(char *e, int &source, int &dest, vector<int> &Vs);
void dijkstra(int s, int d);
path mergePath(path p1, path p2);
bool mySort(const path &p1, const path &p2);

#endif
