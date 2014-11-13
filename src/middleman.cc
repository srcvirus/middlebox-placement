#include "datastructure.h"
#include "util.h"
#include "io.h"
#include "viterbi.h"
#include "cplex4.h"

#include <chrono>
#include <map>
#include <utility>
#include <memory>
#include <stdio.h>
#include <string>
#include <string.h>

const std::string kUsage =
    "./middleman "
    "--per_core_cost=<per_core_cost>\n\t--per_bit_transit_cost=<per_bit_transit"
    "_cost>\n\t--topology_file=<topology_file>\n\t"
    "--middlebox_spec_file=<middlebox_spec_file>\n\t--traffic_r"
    "equest_file=<traffic_request_file>\n\t--algorithm=<algorithm>";

std::vector<middlebox> middleboxes;
std::vector<traffic_class> traffic_classes;
std::vector<traffic_request> traffic_requests;
std::vector<node> nodes;
std::vector<std::vector<edge_endpoint> > graph;
std::vector<std::vector<middlebox_instance> > deployed_mboxes;
double per_core_cost, per_bit_transit_cost;
double cost[MAXN][MAXN];
int pre[MAXN][MAXN];
int shortest_path[MAXN][MAXN], sp_pre[MAXN][MAXN];
std::map<std::pair<int, int>, std::unique_ptr<std::vector<int> > > path_cache;
solution_statistics stats;

int main(int argc, char *argv[]) {
  if (argc < 7) {
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
    } else if (argument.first == "--middlebox_spec_file") {
      InitializeMiddleboxes(argument.second.c_str());
      // PrintMiddleboxes();
    } else if (argument.first == "--traffic_request_file") {
      InitializeTrafficRequests(argument.second.c_str());
    } else if (argument.first == "--algorithm") {
      algorithm = argument.second;
    }
  }
  if (algorithm == "cplex") {
    run_cplex();
  } else if (algorithm == "viterbi") {
    int current_time = traffic_requests[0].arrival_time;
    unsigned long long elapsed_time = 0;
    stats.num_accepted = stats.num_rejected = 0;
    const int kNumTrafficRequests = static_cast<int>(traffic_requests.size());
    for (int i = 0; i < kNumTrafficRequests; ++i) {
      if (current_time != traffic_requests[i].arrival_time) {
	current_time = traffic_requests[i].arrival_time;
	ReleaseAllResources();
      }
      auto solution_start_time = std::chrono::high_resolution_clock::now();
      std::unique_ptr<std::vector<int> > result =
        ViterbiCompute(traffic_requests[i]);
      UpdateResources(result.get(), traffic_requests[i]);
      auto solution_end_time = std::chrono::high_resolution_clock::now();
      elapsed_time += std::chrono::duration_cast<std::chrono::nanoseconds>(
        solution_end_time - solution_start_time).count();
      if ( i % 500 == 0 ) {
	double percentage_completed = 100.0 * static_cast<double>(i) /
	  static_cast<double>(kNumTrafficRequests);
	printf("%.2lf%% traffics completed\n", percentage_completed);
      }
    }
    printf("Solution time: %llu.%llus\n", elapsed_time / ONE_GIG,
	   elapsed_time % ONE_GIG);
    printf("Acceptance Ratio: %.2lf%%\n",
	   100.0 * static_cast<double>(stats.num_accepted) /
	   static_cast<double>(stats.num_accepted + stats.num_rejected));
  }
  return 0;
}