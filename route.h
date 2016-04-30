#ifndef __ROUTE_H__
#define __ROUTE_H__

#include "lib_io.h"
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
#define MAX_NODE_NUM 2000

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

    int compare(path other){
        __gnu_cxx::hash_map<int, int> m;
        int same_edges = 0;
        for(int i=0;i<edges.size();i++){
            m[edges[i]] = 0;
        }
        for(int i=0;i<other.edges.size();i++){
            if(m.find(other.edges[i]) != m.end()) same_edges++;
        }
        return same_edges;
    }

    int count(__gnu_cxx::hash_map<int, int> &node_vs){
        passNodes = 0;
        for(int i=1;i<nodes.size();i++){
            if(node_vs.find(nodes[i]) != node_vs.end()) passNodes++;
        }
        return passNodes;
    }
};

void search_route(char *graph[MAX_EDGE_NUM], int edge_num, char *condition[MAX_DEMAND_NUM], int demand_num);
void resolve(char * e, int* result);
void create_matrix(char *topo[5000], int edge_num, int &node_num, vector< vector<neighbor> > &m);
void resolve_demand(char *e[], int &source, int &dest, vector< vector<int> > &Vs);
void dijkstra(int s, int d, int setflag);
path mergePath(path p1, path p2);
void bad_search_route();
bool mySort(const path &p1, const path &p2);

#endif
