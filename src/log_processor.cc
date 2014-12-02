#include "datastructure.h"
#include "io.h"
#include "util.h"

#include <string>
#include <vector>

std::vector<middlebox> middleboxes;
std::vector<traffic_request> traffic_requests;
std::vector<node> nodes;
std::vector<std::vector<edge_endpoint> > graph;
std::vector<std::vector<middlebox_instance> > deployed_mboxes;
std::vector<double> deployment_costs, energy_costs, transit_costs, sla_costs,
total_costs, stretches;
std::vector<double> net_util;
double per_core_cost, per_bit_transit_cost;
double cost[MAXN][MAXN];
int pre[MAXN][MAXN];
int shortest_path[MAXN][MAXN], sp_pre[MAXN][MAXN];
int shortest_edge_path[MAXN][MAXN];
std::map<std::pair<int, int>, std::unique_ptr<std::vector<int> > > path_cache;
solution_statistics stats;
std::vector<std::unique_ptr<std::vector<int> > > all_results;
std::vector<std::vector<int> > results;
middlebox fake_mbox("switch", "0", "0", TOSTRING(INF), "0.0");

int main(int argc, char *argv[]) {
  auto arg_maps = ParseArgs(argc, argv);
  std::string log_file_prefix = "log";
  for (auto argument : *arg_maps) {
    if (argument.first == "--per_bit_transit_cost") {
      per_bit_transit_cost = atof(argument.second.c_str());
    } else if (argument.first == "--topology_file") {
      InitializeTopology(argument.second.c_str());
    } else if (argument.first == "--middlebox_spec_file") {
      InitializeMiddleboxes(argument.second.c_str());
    } else if (argument.first == "--traffic_request_file") {
      InitializeTrafficRequests(argument.second.c_str());
    } else if (argument.first == "--sequence_file") {
      InitializeAllResults(argument.second.c_str());
    }
    else if (argument.first == "--log_file_prefix") {
      log_file_prefix = argument.second;
    }
  }
  for (int i = 0; i < traffic_requests.size(); ++i) {
    traffic_requests[i].duration = 6000;
  }
  ComputeSolutionCosts(results);
  ComputeAllStretches(results);
  ComputeNetworkUtilization(results);
  ProcessCostLogs(log_file_prefix);
  ProcessStretchLogs(log_file_prefix);
  ProcessNetUtilizationLogs(log_file_prefix);
  return 0;
}

