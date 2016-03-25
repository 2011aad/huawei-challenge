
#include "route.h"

vector< vector<neighbor> > matrix;
__gnu_cxx::hash_map<int, path> minPaths;
vector<int> Vs;
int source, destination, node_num;

//你要完成的功能总入口
void search_route(char *topo[5000], int edge_num, char *demand)
{
    unsigned short result[] = {2, 6, 3};//示例中的一个解
    //use node_num as the number of nodes.
    create_matrix(topo, edge_num, node_num, matrix);
    resolve_demand(demand, source, destination, Vs);

    //find shortest paths between different node pairs.
    int noList1[] = {destination};
    int noList2[] = {source, destination};
    dijkstra(source, noList1, 1); //remove destination when calculating path from source
    for(int i=0;i<Vs.size();i++) dijkstra(Vs[i], noList2, 2);
    //remove source and destination when calculating path between nodes in Vs
    noList1[0] = source;
    //remove source when calculating path from node in Vs to destination
    for(int i=0;i<Vs.size();i++) dijkstra(Vs[i], noList1, 1);

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
    int i=3, j=17;
    for(int k=0;k<minPaths[i*1000+j].nodes.size()-1;k++)
        cout<<minPaths[i*1000+j].nodes[k]<<"->";
    cout<<minPaths[i*1000+j].nodes[minPaths[i*1000+j].nodes.size()-1]<<endl;
    for(int k=0;k<minPaths[i*1000+j].edges.size()-1;k++)
        cout<<minPaths[i*1000+j].edges[k]<<"|";
    cout<<minPaths[i*1000+j].edges[minPaths[i*1000+j].edges.size()-1]<<endl;
    cout<<"from "<<minPaths[i*1000+j].src<<" to "<<minPaths[i*1000+j].dest<<" cost: "<<minPaths[i*1000+j].cost<<endl;

    //-------------------------tests end--------------------------------------------
    for (int i = 0; i < 3; i++)
        record_result(result[i]);
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
    vector<path> ps;
    path p;

    for(int i=0;i<node_num;i++){
        distance[i] = INFINITE;
        known[i] = false;
        prev[i] = i;
        edge[node_num] = -1;
    }
    for(int i=0;i<noListSize;i++) known[noList[i]] = true;
    distance[s] = 0;

    for(int i=1;i<node_num-noListSize;i++){
        min = INFINITE;
        for(int j=0;j<node_num;j++){
            if(!known[j] && distance[j]<min){
                min = distance[j];
                v = j;
            }
        }

        known[v] = true;
        for(int j=0;j<matrix[v].size();j++){
            if(!known[matrix[v][j].vertex] && distance[matrix[v][j].vertex]>min+matrix[v][j].cost){
                distance[matrix[v][j].vertex] = min+matrix[v][j].cost;
                prev[matrix[v][j].vertex] = v;
                edge[matrix[v][j].vertex] = matrix[v][j].LinkID;
            }
        }
    }

    //save path to destination only when calculating path from Vs nodes to destination
    if(s != source && noListSize==1){
        p = path(s, destination, distance[destination]);
        v = destination;
        while(v!=s){
            p.nodes.insert(p.nodes.begin(),v);
            p.edges.insert(p.edges.begin(),edge[v]);
            v = prev[v];
        }
        p.nodes.insert(p.nodes.begin(),v);
        minPaths[s*1000+destination] = p;
    }

    for(int i=0;i<Vs.size();i++){
        if(s==Vs[i]) continue;
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
        minPaths[s*1000+Vs[i]] = p;
    }
}


path mergePath(path p1, path p2){
    path p = path(p1.src, p2.dest, p1.cost+p2.cost);
    for(int i=0;i<p1.nodes.size();i++){
        p.nodes.push_back(p1.nodes[i]);
    }

    for(int i=0;i<p1.edges.size();i++){
        p.edges.push_back(p1.edges[i]);
    }

    for(int i=0;i<p2.nodes.size();i++){
        p.nodes.push_back(p2.nodes[i]);
    }

    for(int i=0;i<p1.edges.size();i++){
        p.edges.push_back(p2.edges[i]);
    }

    return p;
}
