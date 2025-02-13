#include <iostream>
#include <vector>
#include <limits.h>
using namespace std;

// RFID okuma kütüphanesi dahil edilir (gerçek donanıma göre değişir)


#define V 8 // RFID etiket sayısı
#define DISTANCE 1 // Sabit mesafe

// Grafı oluştur (mesafeler sabit)
vector<vector<int>> generateGraph() {
    vector<vector<int>> graph(V, vector<int>(V, 0));

    // Tüm mesafeleri eşit olarak tanımla
    for (int i = 0; i < V; ++i) {
        for (int j = 0; j < V; ++j) {
            if (i != j) {
                graph[i][j] = DISTANCE; // Tüm bağlantılar eşit mesafede
            }
        }
    }

    graph={{0,1,0,0,0,1,0,0},{1,0,1,0,0,0,0,0,},
            {0,1,0,1,1,0,0,0},{0,0,1,0,0,1,1,1},
            {0,0,1,0,0,0,1,0},{1,0,0,1,0,0,0,1},
            {0,0,0,1,1,0,0,0},{0,0,0,1,0,1,0,0}};

    return graph;
}

// Dijkstra algoritması
int minDistance(int dist[], bool sptSet[]) {
    int min = INT_MAX, min_index;

    for (int v = 0; v < V; v++)
        if (!sptSet[v] && (dist[v] <= min))
            min = dist[v], min_index = v;

    return min_index;
}

void printSolution(int dist[]) {
    cout << "Vertex \t Distance from Source" << endl;
    for (int i = 0; i < V; i++)
        cout << i << " \t\t\t\t" << dist[i] << endl;
}

void dijkstra(const vector<vector<int>>& graph, int src) {
    int dist[V];
    bool sptSet[V];

    for (int i = 0; i < V; i++)
        dist[i] = INT_MAX, sptSet[i] = false;

    dist[src] = 0;

    for (int count = 0; count < V - 1; count++) {
        int u = minDistance(dist, sptSet);
        sptSet[u] = true;

        for (int v = 0; v < V; v++)
            if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX
                && dist[u] + graph[u][v] < dist[v])
                dist[v] = dist[u] + graph[u][v];
    }

    printSolution(dist);
}

int main() {


    // Grafı oluştur
    vector<vector<int>> graph = generateGraph();

    // Dijkstra algoritmasını çağır
    dijkstra(graph, 0);

    return 0;
}



/*

001100
001100
001100
001100
001100
111111
000000







*/