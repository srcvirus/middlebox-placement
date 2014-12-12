#include <stdio.h>
#include <string.h>
#include <string>
#include <stdlib.h>
#include <map>
#include <memory.h>

#define MAXN 100
#define ARR_SIZE(arr) (sizeof(arr) / sizeof(arr[0]))

bool IsEdgeNode(const std::string& node) {
  return node[0] == 'e' && node[1] == 's';
}

int main(int argc, char* argv[]) {
  const char* topology_file_name = argv[1];
  FILE* topology_file = fopen(topology_file_name, "r");
  FILE* output_file = fopen("univ1.topo", "w");

  using std::map;
  using std::string;

  map<string,int> node_map;
  long adj[MAXN][MAXN];
  bool edge_node[MAXN];
  memset(adj, 0, sizeof(adj));
  memset(edge_node, false, sizeof(edge_node));

  char buffer[512] = {0};
  int node_count = 0;
  int edge_count = 0;
  int capacities[] = {0, 32, 32, 0, 32, 32, 0, 32, 32, 0, 32};

  while(fgets(buffer, sizeof(buffer), topology_file)) {
    string str_u = string(strtok(buffer, " "));
    string bw_string = string(strtok(NULL, " "));
    string str_v = string(strtok(NULL, " "));
    if (!node_map[str_u]) node_map[str_u] = node_count++;
    if (!node_map[str_v]) node_map[str_v] = node_count++;
    long bw = 1000000000000L;
    if (bw_string[0] == 'T' && bw_string[1] == 'e' && bw_string[2] == 'n')
      bw *= 10;
    int U = node_map[str_u], V = node_map[str_v];
    edge_node[U] = IsEdgeNode(str_u);
    edge_node[V] = IsEdgeNode(str_v);
    adj[U][V] = adj[V][U] = bw;
  }
  for (int i = 0; i < node_count; ++i) {
    for (int j = i + 1; j < node_count; ++j) {
      if (adj[i][j]) ++edge_count;
    }
  }
  fprintf(output_file, "%d %d\n", node_count, edge_count);
  for (int i = 0; i < node_count; ++i) {
    int capacity = 0;
    if (edge_node[i]) {
      capacity = capacities[rand() % ARR_SIZE(capacities)];
    }
    fprintf(output_file, "%d %d\n", i, capacity);
  }
  for (int i = 0; i < node_count; ++i) {
    for (int j = i + 1; j < node_count; ++j) {
      if (adj[i][j]) {
        int latency = 1;
        fprintf(output_file, "%d %d %ld %d\n", i, j, adj[i][j], latency);
      }
    }
  }
  fclose(output_file);
  FILE* sw_prop = fopen("univ1.topo.sw_prop", "w");
  for (int i = 0; i < node_count; ++i)
    fprintf(sw_prop, "%d\n", edge_node[i]);
  fclose(sw_prop);
  return 0;
}
