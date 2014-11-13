#include "datastructure.h"
#include "util.h"
#include "io.h"
#include "viterbi.h"
#include "cplex4.h"

#include <map>
#include <utility>
#include <memory>
#include <stdio.h>
#include <string>
#include <string.h>

const std::string kUsage =
    "./middleman "
    "--per_core_cost=<per_core_cost>\n\t--per_bit_transit_cost=<per_bit_transit"
    "_cost>\n\t--topology_file=<topology_file>\n\t--traffic_class_file=<traffic"
    "_class_file>\n\t--middlebox_spec_file=<middlebox_spec_file>\n\t--traffic_r"
    "equest_file=<traffic_request_file>\n\t--algorithm=<algorithm>";
std::vector<middlebox> middleboxes;
std::vector<traffic_class> traffic_classes;
std::vector<traffic_request> traffic_requests;
std::vector<node> nodes;
std::vector<std::vector<edge_endpoint> > graph;
double per_core_cost, per_bit_transit_cost;
double cost[MAXN][MAXN];
int pre[MAXN][MAXN];
int shortest_path[MAXN][MAXN], sp_pre[MAXN][MAXN];
std::map<std::pair<int, int>, std::unique_ptr<std::vector<int> > > path_cache;

int main(int argc, char *argv[]) {
  if (argc < 8) {
    puts(kUsage.c_str());
    return 1;
  }
  auto arg_maps = ParseArgs(argc, argv);
  string algorithm;
  for (auto argument : *arg_maps) {
    if (argument.first == "--per_core_cost") {
      per_core_cost = atof(argument.second.c_str());
    } else if (argument.first == "--per_bit_transit_cost") {
      per_bit_transit_cost = atof(argument.second.c_str());
    } else if (argument.first == "--topology_file") {
      InitializeTopology(argument.second.c_str());
    } else if (argument.first == "--traffic_class_file") {
      InitializeTrafficClasses(argument.second.c_str());
      PrintTrafficClasses();
    } else if (argument.first == "--middlebox_spec_file") {
      InitializeMiddleboxes(argument.second.c_str());
      PrintMiddleboxes();
    } else if (argument.first == "--traffic_request_file") {
      InitializeTrafficRequests(argument.second.c_str());
      PrintTrafficRequests();
    } else if (argument.first == "--algorithm") {
      algorithm = argument.second;
    }
  }

  if (algorithm == "viterbi") {
    for (int i = 0; i < traffic_requests.size(); ++i) {
      std::unique_ptr<std::vector<int> > result =
              ViterbiCompute(traffic_requests[i]);
      UpdateResources(result.get(), traffic_requests[i]);
      for (int j = 0; j < result->size(); ++j) {
        printf(" %d", result->at(j));
      }
      printf("\n");
    }
  } else if (algorithm == "cplex") {
    run_cplex();
  }
 
  return 0;
}
