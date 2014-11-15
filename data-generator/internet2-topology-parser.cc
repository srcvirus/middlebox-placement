#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>
#include <map>

using std::vector;
using std::string;
using std::map;

// Get the size of a statically declared array.
#define ARR_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

int main(int argc, char* argv[]) {
  const char* topology_file_name = argv[1];
  const char* output_file_name = "Internet2.topo";
  FILE* fp = fopen(topology_file_name, "r");
  FILE* ofp = fopen(output_file_name, "w");
  const int kNodeCount = 12, kEdgeCount = 15;
  
  // Possible numebr of cores.
  int capacities[] = {0, 4, 6, 0, 8, 12, 0, 16, 32, 0, 64};
  
  map <string,int> node_dictionary;
  fprintf(ofp, "%d %d\n", kNodeCount, kEdgeCount);
  for (int i = 0; i < kNodeCount; ++i) {
    double lat, lon;
    char router_name[64];
    char router_location[64];
    fscanf(fp, "%s %s %lf %lf", router_name, router_location, &lat, &lon);
    node_dictionary[std::string(router_name)] = i;
    fprintf(ofp, "%d %d\n", i, capacities[rand() % ARR_SIZE(capacities)]);
  }
  bool visited[kNodeCount + 1][kNodeCount + 1] = {{false}};
  for (int i = 0; i < kEdgeCount * 2; ++i) {
    char router_a[64], router_b[64];
    int bw, ospf_w;
    fscanf(fp, "%s %s %d %d", router_a, router_b, &bw, &ospf_w);
    int u = node_dictionary[std::string(router_a)];
    int v = node_dictionary[std::string(router_b)];
    if (visited[u][v] || visited[v][u]) continue;
    visited[u][v] = true;
    if (ospf_w > 10) ospf_w /= 10;
    fprintf(ofp, "%d %d %d %d\n", u, v, bw, ospf_w);
  }
  return 0;
}
