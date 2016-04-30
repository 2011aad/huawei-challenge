#include "route.h"

#define key(a, b) (a*MAX_NODE_NUM+b)

vector< vector<neighbor> > matrix;
__gnu_cxx::hash_map<int, path> minPaths;
__gnu_cxx::hash_map<int, int> node_vs[MAX_DEMAND_NUM];
vector< vector<int> > Vs;
int source, destination, node_num, total_weight;

//你要完成的功能总入口
void search_route(char *topo[MAX_EDGE_NUM], int edge_num, char *demand[MAX_DEMAND_NUM], int demand_num)
{
    // unsigned short result1[] = {0, 1, 2};//P'路径
    // unsigned short result2[] = {5, 6, 2};//P''路径
    int total_path_num = 1000;
    int final_path_num = 100;
    int MIN_PATH_NUM = 20;
    //use node_num as the number of nodes.
    create_matrix(topo, edge_num, node_num, matrix);
    resolve_demand(demand, source, destination, Vs);
    cout<<Vs[0].size()<<' '<<Vs[1].size()<<endl;

    for(int i=0;i<MAX_DEMAND_NUM;i++){
        for(int j=0;j<Vs[i].size();j++){
            dijkstra(source, Vs[i][j],i);
            node_vs[i][Vs[i][j]] = j;
            // if(minPaths.find(key(source,Vs[i][j])) != minPaths.end())
            //     minPaths[key(source,Vs[i][j])].printPath();
        }
    }

    for(int k=0;k<MAX_DEMAND_NUM;k++){
        for(int i=0;i<Vs[k].size();i++){
            for(int j=0;j<Vs[k].size();j++){
                if(j==i) continue;
                else{
                    dijkstra(Vs[k][i], Vs[k][j], k);
                    // if(minPaths.find(key(Vs[i],Vs[j])) != minPaths.end())
                    //     minPaths[key(Vs[i],Vs[j])].printPath();
                }
            }
        }
    }


    for(int i=0;i<MAX_DEMAND_NUM;i++){
        for(int j=0;j<Vs[i].size();j++){
            dijkstra(Vs[i][j], destination ,i);
            //node_vs[Vs[i]] = i;
            // if(minPaths.find(key(source,Vs[i][j])) != minPaths.end())
            //     minPaths[key(source,Vs[i][j])].printPath();
        }
    }

    // for(int i=0;i<node_num;i++){
    //     cout<<"neighbor for node "<<i<<':'<<endl;
    //     for(int j=0;j<matrix[i].size();j++){
    //         cout<<matrix[i][j].vertex<<':'<<matrix[i][j].cost<<' ';
    //     }
    //     cout<<endl;
    // }
    // cout<<"source: "<<source<<endl;
    // cout<<"destination: "<<destination<<endl;
    // for(int i=0;i<MAX_DEMAND_NUM;i++){
    //     cout<<"demand: "<<demand[i]<<endl;
    //     for(int j=0;j<Vs[i].size();j++)
    //         cout<<Vs[i][j]<<' ';
    //     cout<<endl;
    // }
    // cout<<endl;

    vector< vector<path> > iteration_old, iteration_new;
    vector<path> pathsToSpeNode;
    vector<path> pathset[MAX_DEMAND_NUM];
    path p, tmp;
    for(int d=0;d<MAX_DEMAND_NUM;d++){
        iteration_old.clear();
        iteration_new.clear();
        for(int i=0;i<Vs.size();i++){
            pathsToSpeNode.clear();
            if(minPaths.find(key(source,Vs[d][i])) != minPaths.end()){
                p = minPaths[key(source,Vs[d][i])];
                if(p.isLoopless()){
                    pathsToSpeNode.push_back(p);
                }
            }
            iteration_new.push_back(pathsToSpeNode);
        }

        for(int it=1;it<Vs[d].size() && iteration_new.size()>0;it++){
            cout<<"iteration: "<<it<<':'<<endl;

            int t = 0;
            for(int i=0;i<iteration_new.size();i++) t += iteration_new[i].size();
            cout<<"           queue length: "<<t<<endl;

            iteration_old.clear();
            pathsToSpeNode.clear();
            for(int i=0;i<Vs[d].size();i++) iteration_old.push_back(pathsToSpeNode);

            if(t>total_path_num){
                for(int i=0;i<iteration_new.size();i++){
                    if(iteration_new[i].size()>MIN_PATH_NUM){
                        sort(iteration_new[i].begin(),iteration_new[i].end(),mySort);
                        int j;
                        for(j=0;j<MIN_PATH_NUM;j++){
                            iteration_old[i].push_back(iteration_new[i][j]);
                        }
                        for(;j<iteration_new[i].size();j++) pathsToSpeNode.push_back(iteration_new[i][j]);
                    }
                    else iteration_old[i] = iteration_new[i];
                    iteration_new[i].clear();
                }
                if(pathsToSpeNode.size()>total_path_num){
                    sort(pathsToSpeNode.begin(),pathsToSpeNode.end(),mySort);
                    pathsToSpeNode.erase(pathsToSpeNode.begin()+total_path_num, pathsToSpeNode.end());
                }
                for(int i=0;i<pathsToSpeNode.size();i++){
                    iteration_old[node_vs[d][pathsToSpeNode[i].dest]].push_back(pathsToSpeNode[i]);
                }
            }
            else{
                iteration_old = iteration_new;
            }

            iteration_new.clear();
            pathsToSpeNode.clear();
            for(int i=0;i<Vs[d].size();i++) iteration_new.push_back(pathsToSpeNode);

            for(int i=0;i<iteration_old.size();i++){
                for(int k=0;k<iteration_old[i].size();k++){
                    for(int j=0;j<Vs[d].size();j++){
                        if(iteration_old[i][k].dest==Vs[d][j]) continue;
                        if(minPaths.find(key(iteration_old[i][k].dest, Vs[d][j])) != minPaths.end()){
                            p = mergePath(iteration_old[i][k], minPaths[key(iteration_old[i][k].dest, Vs[d][j])]);
                            if(p.isLoopless() && p.passNodes==it) iteration_new[j].push_back(p);
                        }
                    }
                }
            }
        }

        for(int i=0;i<iteration_new.size();i++){
            if(iteration_new[i].size()==0 || minPaths.find(key(iteration_new[i][0].dest, destination)) == minPaths.end()) continue;
            for(int j=0;j<iteration_new[i].size();j++){
                tmp = mergePath(iteration_new[i][j], minPaths[key(iteration_new[i][j].dest, destination)]);
                if(tmp.isLoopless()){
                    pathset[d].push_back(tmp);
                }
            }
        }

        if(pathset[d].size()>final_path_num){
            sort(pathset[d].begin(),pathset[d].end(),mySort);
            pathset[d].erase(pathset[d].begin()+final_path_num, pathset[d].end());
        }
    }

    cout<<pathset[0].size()<<' '<<pathset[1].size()<<endl;
    int min_same_edge = INFINITE, min_cost = INFINITE;
    int min_i = 0, min_j = 0;
    for(int i=0;i<pathset[0].size();i++){
        for(int j=0;j<pathset[1].size();j++){
            int same_edges = pathset[0][i].compare(pathset[1][j]);
            if(same_edges<min_same_edge){
                min_same_edge = same_edges;
                min_cost = pathset[0][i].cost + pathset[1][j].cost;
                min_i = i;
                min_j = j;
            }
            else if(same_edges==min_same_edge){
                if(pathset[0][i].cost + pathset[1][j].cost< min_cost){
                    min_cost = pathset[0][i].cost + pathset[1][j].cost;
                    min_i = i;
                    min_j = j;
                }
            }
        }
    }
    // cout<<min_i<<' '<<min_j<<endl;


    // p = mergePath(minPaths[key(0,2)], minPaths[key(2,3)]);
    // if(p.isLoopless()) cout<<"loopless"<<endl;
    // else cout<<"has loop"<<endl;

    //
    // if(p.src==source && p.dest==destination && p.passNodes==Vs.size()) {
    //     //p.printPath();
    //     for (int i = 0; i < p.edges.size(); i++)
    //         record_result(p.edges[i]);
    // }
    cout<<"Path 1: "<<endl;
    pathset[0][min_i].printPath();
    for (int i = 0; i < pathset[0][min_i].edges.size(); i++)
    {
        record_result(WORK_PATH, pathset[0][min_i].edges[i]);
    }
    cout<<"Path 2: "<<endl;
    pathset[1][min_j].printPath();
    for (int i = 0; i < pathset[1][min_j].edges.size(); i++)
    {
        record_result(BACK_PATH, pathset[1][min_j].edges[i]);
    }
    cout<<"Same edges: "<<min_same_edge<<endl;
    cout<<"Min Cost: "<<min_cost<<endl;
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
void resolve_demand(char *e[], int &source, int &dest, vector< vector<int> > &Vs){
    int i, tmp = 0;
    vector<int> V;
    for(int j=0;j<MAX_DEMAND_NUM;j++){
        tmp = 0;
        V.clear();
        for(i=0;e[j][i]!=',';i++);
        i++;
        for(source = 0;e[j][i]!=',';i++){
            source *= 10;
            source += (int)(e[j][i]-'0');
        }

        for(dest = 0, i++;e[j][i]!=',';i++){
            dest *= 10;
            dest += (int)(e[j][i]-'0');
        }

        for(i++;e[j][i] && e[j][i]>='0';i++){
            if(e[j][i]=='|'){
                V.push_back(tmp);
                tmp = 0;
            }
            else{
                tmp *= 10;
                tmp += (int)(e[j][i]-'0');
            }
        }
        V.push_back(tmp);
        Vs.push_back(V);
    }
}


void dijkstra(int s, int d, int setflag){
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
        edge[i] = -1;
    }
    for(int i=0;i<Vs[setflag].size();i++){
        if(Vs[setflag][i] != s && Vs[setflag][i] != d) known[Vs[setflag][i]] = true;
    }
    if(s != source) known[source] = true;
    if(d != destination) known[destination] = true;

    distance[s] = 0;

    for(int i=1;i<node_num-Vs[setflag].size();i++){
        min = INFINITE+1;
        for(int j=0;j<node_num;j++){
            if(!known[j] && distance[j]<min){
                min = distance[j];
                v = j;
            }
        }

        if(v==d) break;
        known[v] = true;
        for(int j=0;j<matrix[v].size();j++){
            if(distance[v]>=INFINITE) break;
            if(!known[matrix[v][j].vertex] && distance[matrix[v][j].vertex]>min+matrix[v][j].cost){
                distance[matrix[v][j].vertex] = min+matrix[v][j].cost;
                prev[matrix[v][j].vertex] = v;
                edge[matrix[v][j].vertex] = matrix[v][j].LinkID;
            }
        }
    }

    p = path(s, d, distance[d]);
    v = d;
    while(v!=s){
        p.nodes.insert(p.nodes.begin(),v);
        p.edges.insert(p.edges.begin(),edge[v]);
        if(v==prev[v]) return;
        v = prev[v];
    }
    p.nodes.insert(p.nodes.begin(),s);
    minPaths[key(s,d)] = p;
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
    p.passNodes = p1.passNodes + 1;

    return p;
}

bool mySort(const path &p1, const path &p2){
    return p1.cost<p2.cost;
}
