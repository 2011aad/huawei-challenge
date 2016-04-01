
#include "route.h"

#define key(a, b) (a*1000+b)

vector< vector<neighbor> > matrix;
__gnu_cxx::hash_map<int, path> minPaths;
__gnu_cxx::hash_map<int, int> node_vs;
vector<int> Vs;
int source, destination, node_num, total_weight;

//你要完成的功能总入口
void search_route(char *topo[5000], int edge_num, char *demand)
{
    //use node_num as the number of nodes.
    create_matrix(topo, edge_num, node_num, matrix);
    resolve_demand(demand, source, destination, Vs);
    int path_num;
    if(Vs.size()<=10) path_num = 1000;
    else if(Vs.size()<=17) path_num = 800;
    else path_num = 200;

    //for(int i=0;i<Vs.size();i++) node_vs[Vs[i]] = 0;
    //find shortest paths between different node pairs.
    for(int i=0;i<Vs.size();i++){
        dijkstra(source, Vs[i]);
        // if(minPaths.find(key(source,Vs[i])) != minPaths.end())
        //     minPaths[key(source,Vs[i])].printPath();
    }

    for(int i=0;i<Vs.size();i++){
        for(int j=0;j<Vs.size();j++){
            if(j==i) continue;
            else{
                dijkstra(Vs[i], Vs[j]);
                // if(minPaths.find(key(Vs[i],Vs[j])) != minPaths.end())
                //     minPaths[key(Vs[i],Vs[j])].printPath();
            }
        }
    }

    for(int i=0;i<Vs.size();i++){
        dijkstra(Vs[i], destination);
        // if(minPaths.find(key(Vs[i], destination)) != minPaths.end())
        //     minPaths[key(Vs[i], destination)].printPath();
    }

    vector<path> iteration_old, iteration_new;
    path p, tmp;
    for(int i=0;i<Vs.size();i++){
        if(minPaths.find(key(source,Vs[i])) != minPaths.end()){
            p = minPaths[key(source,Vs[i])];
            if(p.isLoopless()) iteration_new.push_back(p);
        }

    }

    for(int it=1;it<Vs.size() && iteration_new.size()>0;it++){
        // cout<<"iteration: "<<it<<':'<<endl;
        // cout<<"           queue length: "<<iteration_new.size()<<endl;
        iteration_old = iteration_new;
        iteration_new.clear();
        for(int i=0;i<iteration_old.size();i++){
            if(iteration_old.size()>path_num){
                sort(iteration_old.begin(),iteration_old.end(),mySort);
                iteration_old.erase(iteration_old.begin()+path_num, iteration_old.end());
            }
            for(int j=0;j<Vs.size();j++){
                if(iteration_old[i].dest==Vs[j]) continue;
                if(minPaths.find(key(iteration_old[i].dest, Vs[j])) != minPaths.end()){
                    p = mergePath(iteration_old[i], minPaths[key(iteration_old[i].dest, Vs[j])]);
                    if(p.isLoopless() && p.passNodes==it) iteration_new.push_back(p);
                }
            }
        }
    }

    int min = INFINITE;
    if(iteration_new.size()==0) return;
    p = path(source, source ,0);
    for(int i=0;i<iteration_new.size();i++){
        if(minPaths.find(key(iteration_new[i].dest, destination)) != minPaths.end()){
            if(iteration_new[i].cost + minPaths[key(iteration_new[i].dest, destination)].cost < min){
                tmp = mergePath(iteration_new[i], minPaths[key(iteration_new[i].dest, destination)]);
                if(tmp.isLoopless()){
                    p = tmp;
                    min = p.cost;
                }
            }
        }
    }


    // p = mergePath(minPaths[key(0,2)], minPaths[key(2,3)]);
    // if(p.isLoopless()) cout<<"loopless"<<endl;
    // else cout<<"has loop"<<endl;


    if(p.src==source && p.dest==destination && p.passNodes==Vs.size()) {
        //p.printPath();
        for (int i = 0; i < p.edges.size(); i++)
            record_result(p.edges[i]);
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


void dijkstra(int s, int d){
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
    for(int i=0;i<Vs.size();i++){
        if(Vs[i] != s && Vs[i] != d) known[Vs[i]] = true;
    }
    if(s != source) known[source] = true;
    if(d != destination) known[destination] = true;

    distance[s] = 0;

    for(int i=1;i<node_num-Vs.size();i++){
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
    p.middle = p2.src;

    return p;
}

bool mySort(const path &p1, const path &p2){
    return p1.cost<p2.cost;
}
