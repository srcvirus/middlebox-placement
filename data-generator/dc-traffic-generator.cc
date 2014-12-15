#include <set>
#include <stdio.h>
#include <string.h>
#include <string>
#include <stdlib.h>
#include <vector>
#include <map>
#include <utility>

#define MAXN 25
const static std::string middlebox_names[] = {
  "firewall", "proxy", "ids", "nat"
};

std::vector<int> RandomSeqWithUniqElements(int n, int N) {
  std::vector<int> ret;
  while(n) {
    bool found = false;
    int current = rand() % N;
    for (int val : ret) {
      if (val == current) {
        found = true;
        break;
      }
    }
    if (!found) {
      ret.push_back(current);
      --n;
    }
  }
  return ret;
}

int StringToIp(std::string str_ip) {
  int ip = 0;
  char buf[20];
  strcpy(buf, str_ip.c_str());
  char *token = strtok(buf, ".");
  ip |= atoi(token);
  while ((token = strtok(NULL, "."))) {
    ip <<= 8;
    ip |= atoi(token);
  }
  return ip;
}

int hash_function(int ip, const std::vector<int> &edge_switches) {
  return edge_switches[ip % edge_switches.size()];
}

int main(int argc, char *argv[]) {
  int num_switches = atoi(argv[1]);
  const char *sw_prop_file = argv[2];
  int is_edge_node[MAXN], index = 0;

  FILE *sw_prop = fopen(sw_prop_file, "r");
  while (fscanf(sw_prop, "%d", &is_edge_node[index++]) != EOF)
    ;
  fclose(sw_prop);
  std::vector<int> edge_nodes;
  for (int i = 0; i < num_switches; ++i) {
    if (is_edge_node[i])
      edge_nodes.push_back(i);
  }
  FILE* ofp = fopen("traffic-request.dc", "w");
  for (int t = 0; t < 9; ++t) {
    int tm[MAXN][MAXN];
    memset(tm, 0, sizeof(tm));
    std::string file_name = "../dc-data/tm_" + std::to_string(t + 1);
    FILE *tm_file = fopen(file_name.c_str(), "r");
    char line[512];
    while (fgets(line, sizeof(line), tm_file)) {
      std::string source = std::string(strtok(line, ","));
      std::string dest = std::string(strtok(NULL, ","));
      int volume = atoi(strtok(NULL, ","));
      int source_ip = StringToIp(source);
      int dest_ip = StringToIp(dest);
      tm[hash_function(source_ip, edge_nodes)][
          hash_function(dest_ip, edge_nodes)] += volume;
    }
    for (int i = 0; i < num_switches; ++i) {
      for (int j = 0; j < num_switches; ++j) {
        if ( i == j ) continue;
        if (tm[i][j] < 800000) continue;
        int u = i, v = j;
        int min_bandwidth = (static_cast<double>(tm[i][j]) / 1000.0) + 0.5;
        int max_latency = 100 + rand() % 100;
        std::vector<int> mbox_seq = RandomSeqWithUniqElements(3, 4);
        double penalty = 0.00000001;
        fprintf(ofp, "%d,%d,%d,%d,%d,%.8lf,%s,%s,%s\n", 5 * t, u, v,
                  min_bandwidth, max_latency, penalty, 
                  middlebox_names[mbox_seq[0]].c_str(),
                  middlebox_names[mbox_seq[1]].c_str(),
                  middlebox_names[mbox_seq[2]].c_str());
      }
    } 
  }
  return 0;
}
