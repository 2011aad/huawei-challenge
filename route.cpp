
#include "route.h"

#define key(a, b) (a*1000+b)

vector< vector<neighbor> > matrix;
__gnu_cxx::hash_map<int, path> minPaths;
__gnu_cxx::hash_map<int, int> node_vs;
vector<int> Vs;
int source, destination, node_num;

//你要完成的功能总入口
void search_route(char *topo[5000], int edge_num, char *demand)
{
    //use node_num as the number of nodes.
    create_matrix(topo, edge_num, node_num, matrix);
    resolve_demand(demand, source, destination, Vs);

    for(int i=0;i<Vs.size();i++) node_vs[Vs[i]] = 0;
    //find shortest paths between different node pairs.
    int noList1[1] = {destination};
    int noList2[2] = {source,destination};
    dijkstra(source, noList1, 1); //remove destination when calculating path from source
    //remove source and destination when calculating path between nodes in Vs
    for(int i=0;i<Vs.size();i++) dijkstra(Vs[i], noList2, 2);
    //remove source when calculating path from node in Vs to destination
    noList1[0] = source;
    for(int i=0;i<Vs.size();i++) dijkstra(Vs[i], noList1, 1);
    //cout<<"Find shortest path between nodes success!"<<endl;

    //vector< vector< vector<path> > >looplessPaths;
    vector< vector<path> > iteration_old, iteration_new;
    vector<path> tmp;
    path p, min_p;
    __gnu_cxx::hash_map<int, int> order_old, order_new;
    // p = mergePath(minPaths[key(0,2)], minPaths[key(2,3)]);
    // if(p.isLoopless()) cout<<"loopless"<<endl;
    // else cout<<"has loop"<<endl;
    int mins[RETAIN_LEVEL];
    path min_ps[RETAIN_LEVEL];

    for(int i=0;i<Vs.size();i++){
        p = minPaths[key(Vs[i],destination)];
        if(p.isLoopless()){
            tmp.push_back(p);
            iteration_new.push_back(tmp);
            order_new[iteration_new.size()-1] = Vs[i];
        }
        //looplessPaths.push_back(iteration_new);
    }

    for(int i=1;i<Vs.size();i++){           //iteration i
        //iteration_old = looplessPaths[i-1];
      //  cout<<"iteration "<<i<<": "<<endl;
        iteration_old = iteration_new;
        iteration_new.clear();
        order_old = order_new;
        order_new.clear();
        for(int node_i=0;node_i<Vs.size();node_i++){
            tmp.clear();
            for(int node_l=0;node_l<iteration_old.size();node_l++){
                if(order_old[node_l]==Vs[node_i]){
                    for(int path_j=0;path_j<iteration_old[node_l].size();path_j++){
                        if(iteration_old[node_l][path_j].passNodes>=i) tmp.push_back(iteration_old[node_l][path_j]);
                    }
                    continue;
                }
                for(int k=0;k<RETAIN_LEVEL;k++){
                    mins[k] = INFINITE;
                    min_ps[k] = path(Vs[node_i], Vs[node_i], mins[k]);
                }
                for(int path_j=0;path_j<iteration_old[node_l].size();path_j++){
                    if(Vs[node_i]==iteration_old[node_l][path_j].middle) continue;
                    p = mergePath(minPaths[key(Vs[node_i],order_old[node_l])], iteration_old[node_l][path_j]);
                    if(p.isLoopless()){
                        int k=0;
                        for(;k<RETAIN_LEVEL;k++){
                            if(p.cost<mins[k]){
                                break;
                            }
                        }
                        if(k>=RETAIN_LEVEL) continue;
                        for(int l=RETAIN_LEVEL-1;l>k;l--){
                            mins[l] = mins[l-1];
                            min_ps[l] = min_ps[l-1];
                        }
                        mins[k] = p.cost;
                        min_ps[k] = p;
                    }
                }
                for(int k=0;k<RETAIN_LEVEL;k++)
                    if(min_ps[k].src != min_ps[k].dest){
                        tmp.push_back(min_ps[k]);
                        //min_p[k].printPath();
                    }
            }
            iteration_new.push_back(tmp);
            order_new[iteration_new.size()-1] = Vs[node_i];
        }
    }

    int min = INFINITE;
    min_p = path(source, source, min);
    for(int node_i=0;node_i<iteration_new.size();node_i++){
        for(int path_l=0;path_l<iteration_new[node_i].size();path_l++){
            p = mergePath(minPaths[key(source,iteration_new[node_i][path_l].src)], iteration_new[node_i][path_l]);
            if(p.isLoopless() && p.cost<min && p.passNodes==Vs.size()){
                min_p = p;
                min = p.cost;
            }
        }
    }
    if(min_p.src==source && min_p.dest==destination && min_p.passNodes==Vs.size()) {
        min_p.printPath();
        for (int i = 0; i < min_p.edges.size(); i++)
            record_result(min_p.edges[i]);
    }


    //-------------------------tests--------------------------------------------
    // cout<<"source: "<<source<<endl;
    // cout<<"destination: "<<destination<<endl;
    // cout<<"demand: "<<demand<<endl;
    // for(int i=0;i<Vs.size();i++){
    //     cout<<Vs[i]<<' ';
    // }
    // cout<<endl;
    // for(int i=0;i<node_num;i++){
    //     cout<<"neighbor for node "<<i<<':'<<endl;
    //     for(int j=0;j<matrix[i].size();j++){
    //         cout<<matrix[i][j].vertex<<':'<<matrix[i][j].cost<<' ';
    //     }
    //     cout<<endl;
    // }
    // int i=2, j=3;
    // for(int k=0;k<minPaths[key(i,j)].nodes.size()-1;k++)
    //     cout<<minPaths[key(i,j)].nodes[k]<<"->";
    // cout<<minPaths[key(i,j)].nodes[minPaths[key(i,j)].nodes.size()-1]<<endl;
    // for(int k=0;k<minPaths[key(i,j)].edges.size()-1;k++)
    //     cout<<minPaths[key(i,j)].edges[k]<<"|";
    // cout<<minPaths[key(i,j)].edges[minPaths[key(i,j)].edges.size()-1]<<endl;
    // cout<<"from "<<minPaths[key(i,j)].src<<" to "<<minPaths[key(i,j)].dest<<" cost: "<<minPaths[key(i,j)].cost<<endl;

    //-------------------------tests end--------------------------------------------

}

//Create a list of node, with its neighbors as sub-list.
void create_matrix(char *topo[5000], int edge_num, int &node_num, vector< vector<neighbor> > &m){
    vector<neighbor> degrees;
    neighbor n;
    int edge[4];
    for(int i=0;i<MAX_NODE_NUM;i++) m.push_back(degrees);
    for(int i=0;i<edge_num;i++){
        resolve(topo[i], edge);
        n = neighbor(edge[DEST],edge[ID],edge[COST]);
        m[edge[SOURCE]].push_back(n);
        if(edge[SOURCE]>node_num) node_num = edge[SOURCE];
        if(edge[DEST]>node_num) node_num = edge[DEST];
    }
    node_num++;
    //cout<<"Create matrix success!"<<endl;
}

//translate string into int array
void resolve(char *e, int* result){
    int i=-1, j, begin = 0, num = 0, counter = 0;
    while(e[++i]){
        if(e[i]==','){
            for(j=begin,num=0;j<i;j++){
                num *= 10;
                num += (int)(e[j]-'0');
            }
            result[counter++] = num;
            begin = i+1;
        }
    }
    for(j=begin,num=0;e[j]>='0';j++){
        num *= 10;
        num += (int)(e[j]-'0');
    }
    result[counter] = num;
}

//resolve demand information
void resolve_demand(char *e, int &source, int &dest, vector<int> &Vs){
    int i = 0, tmp = 0;
    for(source = 0;e[i]!=',';i++){
        source *= 10;
        source += (int)(e[i]-'0');
    }

    for(dest = 0, i++;e[i]!=',';i++){
        dest *= 10;
        dest += (int)(e[i]-'0');
    }

    for(i++;e[i] && e[i]>='0';i++){
        if(e[i]=='|'){
            Vs.push_back(tmp);
            tmp = 0;
        }
        else{
            tmp *= 10;
            tmp += (int)(e[i]-'0');
        }
    }
    Vs.push_back(tmp);
}


void dijkstra(int s, int noList[], int noListSize){
    int min, v;
    int distance[node_num];
    int prev[node_num];
    int edge[node_num];
    bool known[node_num];
    path p;

    for(int i=0;i<node_num;i++){
        distance[i] = INFINITE;
        known[i] = false;
        prev[i] = i;
        edge[node_num] = -1;
    }
    for(int i=0;i<noListSize;i++) known[noList[i]] = true;

    distance[s] = 0;

    for(int i=1;i<(node_num-noListSize);i++){
        min = INFINITE+1;
        for(int j=0;j<node_num;j++){
            if(!known[j] && distance[j]<min){
                min = distance[j];
                v = j;
            }
        }

        known[v] = true;
        for(int j=0;j<matrix[v].size();j++){
            if(distance[v]<INFINITE && !known[matrix[v][j].vertex] && distance[matrix[v][j].vertex]>min+matrix[v][j].cost){
                distance[matrix[v][j].vertex] = min+matrix[v][j].cost;
                prev[matrix[v][j].vertex] = v;
                edge[matrix[v][j].vertex] = matrix[v][j].LinkID;
            }
        }
    }

    //save path to destination only when calculating path from Vs nodes to destination
    if(s != source && noListSize==1 && distance[destination]<INFINITE){
        p = path(s, destination, distance[destination]);
        v = destination;
        while(v!=s){
            p.nodes.insert(p.nodes.begin(),v);
            p.edges.insert(p.edges.begin(),edge[v]);
            v = prev[v];
        }
        p.nodes.insert(p.nodes.begin(),v);
        p.count(node_vs);
        minPaths[key(s,destination)] = p;
        return;
    }

    for(int i=0;i<Vs.size();i++){
        if(s==Vs[i] || distance[Vs[i]]>=INFINITE) continue;
        p = path(s, Vs[i], distance[Vs[i]]);
        if(distance[Vs[i]]==INFINITE)
            continue;

        v = Vs[i];
        while(v!=s){
            p.nodes.insert(p.nodes.begin(),v);
            p.edges.insert(p.edges.begin(),edge[v]);
            v = prev[v];
        }
        p.nodes.insert(p.nodes.begin(),v);
        p.count(node_vs);
        minPaths[key(s,Vs[i])] = p;
    }
}


path mergePath(path p1, path p2){
    if(p1.dest != p2.src){
        //cout<<"Merge path Error(cannot merge)!"<<endl;
        return path(p1.src, p1.src, INFINITE);
    }
    path p = path(p1.src, p2.dest, p1.cost+p2.cost);
    for(int i=0;i<p1.nodes.size();i++){
        p.nodes.push_back(p1.nodes[i]);
    }

    for(int i=0;i<p1.edges.size();i++){
        p.edges.push_back(p1.edges[i]);
    }

    for(int i=1;i<p2.nodes.size();i++){
        p.nodes.push_back(p2.nodes[i]);
    }

    for(int i=0;i<p2.edges.size();i++){
        p.edges.push_back(p2.edges[i]);
    }
    p.passNodes = p1.passNodes + p2.passNodes;
    p.middle = p2.src;

    return p;
}
