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
std::vector<double> deployment_costs, energy_costs, transit_costs,
sla_costs;
double per_core_cost, per_bit_transit_cost;
double cost[MAXN][MAXN];
int pre[MAXN][MAXN];
int shortest_path[MAXN][MAXN], sp_pre[MAXN][MAXN];
std::map<std::pair<int, int>, std::unique_ptr<std::vector<int> > > path_cache;
solution_statistics stats;
std::vector<std::unique_ptr<std::vector<int> > > all_results;

int main(int argc, char *argv[]) {
  if (argc < 6) {
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
    std::vector<traffic_request> current_traffic_requests;
    int current_time = traffic_requests[0].arrival_time;
    double opex, running_time;
    //file to write opex and running time on each arrival time
    FILE* tFile = fopen("time_opex_runtime", "w");
    for (int i = 0; i < traffic_requests.size(); ) {
      traffic_requests[i].duration = 300;
      fprintf(tFile, "%d ", current_time);
      for (;current_time == traffic_requests[i].arrival_time; ++i) {
        current_traffic_requests.push_back(traffic_requests[i]);
      }
      current_time = traffic_requests[i].arrival_time;
      run_cplex(current_traffic_requests, opex, running_time);
      current_traffic_requests.clear();
      fprintf(tFile, "%lf %lf\n", opex, running_time);
      fflush(tFile);
      //cout << "Done with one iteration" << endl;
      //int foo;
      //cin >> foo;
    }
    fclose(tFile);
  } else if (algorithm == "viterbi") {
    int current_time = traffic_requests[0].arrival_time;
    unsigned long long elapsed_time = 0;
    stats.num_accepted = stats.num_rejected = 0;
    const int kNumTrafficRequests = static_cast<int>(traffic_requests.size());
    for (int i = 0; i < kNumTrafficRequests; ++i) {
      traffic_requests[i].duration = 300;
      if (current_time != traffic_requests[i].arrival_time) {
        RefreshServerStats(current_time);
        current_time = traffic_requests[i].arrival_time;
        ReleaseAllResources();
      }

      // Get solution for one traffic.
      auto solution_start_time = std::chrono::high_resolution_clock::now();
      std::unique_ptr<std::vector<int> > result =
          ViterbiCompute(traffic_requests[i]);
      auto solution_end_time = std::chrono::high_resolution_clock::now();
      elapsed_time += std::chrono::duration_cast<std::chrono::nanoseconds>(
          solution_end_time - solution_start_time).count();

      // Compute the cost components.
      double d_cost = 0.0, e_cost = 0.0, t_cost = 0.0, s_cost = 0.0;
      int prev_node = NIL, current_node = NIL;
      const int kLastIndex = static_cast<int>(result->size()) - 1;
      for (int kk = 1; kk < static_cast<int>(result->size()); ++kk) {
        auto& m_box = middleboxes[traffic_requests[i].middlebox_sequence[kk - 1]];
        current_node = result->at(kk);
        prev_node = result->at(kk - 1);
        if (kk != kLastIndex)
          d_cost += GetDeploymentCost(current_node, m_box, traffic_requests[i]);
        if (kk != kLastIndex) 
          e_cost += GetEnergyCost(current_node, m_box, traffic_requests[i]);
        t_cost += GetTransitCost(prev_node, current_node, traffic_requests[i]);
        s_cost += GetSLAViolationCost(prev_node, current_node,
                                      traffic_requests[i]);
      }
      deployment_costs.push_back(d_cost);
      energy_costs.push_back(e_cost);
      transit_costs.push_back(t_cost);
      sla_costs.push_back(s_cost);

      // Update the system resources.
      UpdateResources(result.get(), traffic_requests[i]);

      // Refresh the per server statistics.
      RefreshServerStats(current_time);

      // Progress bar
      if (i % 500 == 0) {
        double percentage_completed = 100.0 * static_cast<double>(i) /
                                      static_cast<double>(kNumTrafficRequests);
        printf("%.2lf%% traffics completed\n", percentage_completed);
      }
      all_results.push_back(std::move(result));
    }
    
    // Print the solution time.
    printf("Solution time: %llu.%llus\n", elapsed_time / ONE_GIG,
           elapsed_time % ONE_GIG);
    printf("Acceptance Ratio: %.2lf%%\n",
           100.0 * static_cast<double>(stats.num_accepted) /
               static_cast<double>(stats.num_accepted + stats.num_rejected));

    // Process that collected statistics and write to log.
    const std::string kStatsOutputFilePrefix = "log";
    ProcessStats(stats, kStatsOutputFilePrefix);
  }

  // DEBUG: Write all the computed sequences in a file.
  FILE* all_results_file = fopen("log.sequences", "w");
  int row_index = 0;
  for (auto& row : all_results) {
    for (int i = 0; i < row->size(); ++i) {
      fprintf(all_results_file, " %d", row->at(i));
      if ( i == 0 || i == row->size() - 1 ) continue;

    }
    double total_cost = deployment_costs[row_index] + energy_costs[row_index] +
                          transit_costs[row_index] + sla_costs[row_index];
    fprintf(all_results_file, " %.3lf %.3lf %.3lf %.3lf %.3lf\n",
            deployment_costs[row_index], energy_costs[row_index],
            transit_costs[row_index], sla_costs[row_index], total_cost);
    ++row_index;
  }
  fclose(all_results_file);

  return 0;
}
