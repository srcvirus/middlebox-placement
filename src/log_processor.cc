#include "datastructure.h"
#include "io.h"
#include "util.h"

#include <string>
#include <vector>

std::vector<middlebox> middleboxes;
std::vector<traffic_request> traffic_requests;
std::vector<node> nodes;
std::vector<std::vector<edge_endpoint>> graph;
std::vector<double> closeness;
std::vector<std::vector<middlebox_instance>> deployed_mboxes;
std::vector<double> deployment_costs, energy_costs, transit_costs, sla_costs,
    total_costs, stretches;
std::vector<double> e_cost_ts;
std::vector<std::vector<int>> ingress_k, egress_k;
std::vector<std::pair<int, int>> num_active_servers;
std::vector<std::vector<double>> sol_closeness;
std::list<int> mbox_count;
std::vector<int> num_service_points;
std::vector<double> net_util;
double per_core_cost, per_bit_transit_cost;
double cost[MAXN][MAXN];
int pre[MAXN][MAXN];
int shortest_path[MAXN][MAXN], sp_pre[MAXN][MAXN];
int shortest_edge_path[MAXN][MAXN];
long bw[MAXN][MAXN];
int max_time;
std::map<std::pair<int, int>, std::unique_ptr<std::vector<int>>> path_cache;
solution_statistics stats;
std::vector<std::unique_ptr<std::vector<int>>> all_results;
std::vector<std::vector<int>> results;
std::vector<std::vector<int>> paths;
middlebox fake_mbox("switch", "0", "0", TOSTRING(INF), "0.0");

int main(int argc, char *argv[]) {
  auto arg_maps = ParseArgs(argc, argv);
  bool processing_cplex = false;
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
    } else if (argument.first == "--max_time") {
      max_time = atoi(argument.second.c_str());
    } else if (argument.first == "--log_file_prefix") {
      log_file_prefix = argument.second;
    } else if (argument.first == "--cplex_solution_path_file") {
      InitializeSolutionPaths(argument.second.c_str());
      processing_cplex = true;
    }
  }
  for (int i = 0; i < traffic_requests.size(); ++i) {
    //    traffic_requests[i].duration = 6000;
  }
  ComputeSolutionCosts(results);
  ComputeServicePoints(results);
  ComputeCloseness(results);
  if (!processing_cplex) {
    ComputeAllStretches(results);
    ComputeNetworkUtilization(results);
    ComputeKHops(results);
  } else {
    CplexComputeAllStretches(paths);
    CplexComputeNetworkUtilization(paths);
    CplexComputeKHops(results, paths);
  }
  ProcessCostLogs(log_file_prefix);
  ProcessStretchLogs(log_file_prefix);
  ProcessNetUtilizationLogs(log_file_prefix);
  ProcessServerUtilizationLogs(log_file_prefix);
  ProcessKHopsLogs(log_file_prefix);
  ProcessMboxRatio(log_file_prefix);
  ProcessServicePointLogs(log_file_prefix);
  ProcessClosenessLogs(log_file_prefix);
  ProcessActiveServerLogs(log_file_prefix);
  return 0;
}
