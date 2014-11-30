#ifndef MIDDLEBOX_PLACEMENT_SRC_UTIL_H_
#define MIDDLEBOX_PLACEMENT_SRC_UTIL_H_
#include "datastructure.h"

#include <algorithm>
#include <stdarg.h>
#include <stdio.h>
#include <string>
#include <time.h>

#define ONE_GIG 1000000000ULL
#define STRINGIFY(x) #x
#define TOSTRING(x) STRINGIFY(x)
#define AT __FILE__ ":" TOSTRING(__LINE__) " "

#ifdef DBG
#define DEBUG(...) PrintDebugMessage(AT, __VA_ARGS__)
#else
#define DEBUG(...)
#endif

void PrintDebugMessage(const char *location, const char *fmt_string, ...) {
  va_list args;
  va_start(args, fmt_string);
  std::string str = location;
  str += fmt_string;
  vprintf(str.c_str(), args);
  fflush(stdout);
  va_end(args);
}

inline unsigned long long CurrentTimeNanos() {
  timespec ts;
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<unsigned long long>(ts.tv_sec) +
         static_cast<unsigned long long>(ts.tv_nsec);
}

template <class T>
double GetMean(const std::vector<T> &data) {
  T sum = T(0);
  const size_t kNumElements = data.size();
  for (auto &element : data) sum += element;
  return sum / static_cast<T>(kNumElements);
}

template <class T>
T GetNthPercentile(const std::vector<T> &data, int n) {
  std::vector<T> temp_data_buffer = data;
  sort(temp_data_buffer.begin(), temp_data_buffer.end());
  const size_t kNumElements = data.size();
  int rank = n * kNumElements;
  if (rank % 100) {
    rank = (rank / 100) + 1;
  } else
    rank /= 100;
  --rank;
  return temp_data_buffer[rank];
}

template <class T>
std::vector<std::pair<T,double>> GetCDF(
    const std::vector<T> &data) {
  int precision = 1;
  std::vector<T> temp_data_buffer = data;
  if (typeid(temp_data_buffer[0]) == typeid(double)
      || typeid(temp_data_buffer[0]) == typeid(float)) {
    precision = 1000;
  }
  std::map<int, int> cdf;
  for (int i = 0; i < temp_data_buffer.size(); ++i) {
    int bucket_index = temp_data_buffer[i] * precision;
    if (cdf[bucket_index]) cdf[bucket_index]++;
    else cdf[bucket_index] = 1;
  }
  std::map<int,int>::iterator prev = cdf.begin(), current = cdf.begin();
  current++;
  for( ; current != cdf.end(); current++, prev++) {
    current->second += prev->second;
  }
  int total = temp_data_buffer.size();
  std::vector<std::pair<T, double>> ret;
  for ( current = cdf.begin() ; current != cdf.end(); ++current) {
    T first = static_cast<T>(current->first) / static_cast<T>(precision);
    double second = static_cast<double>(current->second) /
                      static_cast<double>(total);
    ret.push_back(std::make_pair(first, second));
  }
  return ret;
}

inline std::unique_ptr<std::vector<int>> ComputeShortestPath(int source,
                                                             int destination) {
  std::unique_ptr<std::vector<int>> path(new std::vector<int>());
  while (destination != NIL) {
    path->push_back(destination);
    destination = sp_pre[source][destination];
  }
  std::reverse(path->begin(), path->end());
  return std::move(path);
}

inline void RefreshServerStats(int timestamp) {
  for (auto &node : nodes) {
    if (node.num_cores > 0) {
      double utilization =
          static_cast<double>(node.num_cores - node.residual_cores) /
          static_cast<double>(node.num_cores);
      stats.server_stats.emplace_back(timestamp, node.node_id, utilization);
    }
  }
}

inline int GetEdgeResidualBandwidth(int source, int destination) {
  for (auto &endpoint : graph[source]) {
    if (endpoint.u->node_id == destination) return endpoint.residual_bandwidth;
  }
  return NIL;
}

inline int GetPathResidualBandwidth(int source, int destination) {
  std::vector<int> *path_ptr = nullptr;
  std::pair<int, int> cache_index(source, destination);
  if (path_cache[cache_index]) {
    path_ptr = path_cache[cache_index].get();
  } else {
    path_cache[cache_index] = ComputeShortestPath(source, destination);
    //    path_cache[cache_index] = std::move(path);
    path_ptr = path_cache[cache_index].get();
  }
  int residual_bandwidth = INF;
  for (int i = 0; i < static_cast<int>(path_ptr->size()) - 1; ++i) {
    DEBUG("edge[%d][%d] = %d\n", path_ptr->at(i), path_ptr->at(i + 1),
          GetEdgeResidualBandwidth(path_ptr->at(i), path_ptr->at(i + 1)));
    residual_bandwidth = std::min(
        residual_bandwidth,
        GetEdgeResidualBandwidth(path_ptr->at(i), path_ptr->at(i + 1)));
  }
  return residual_bandwidth;
}

inline void ReduceEdgeResidualBandwidth(int source, int destination,
                                        int bandwidth) {
  for (auto &endpoint : graph[source]) {
    if (endpoint.u->node_id == destination)
      endpoint.residual_bandwidth -= bandwidth;
  }
}

void DecommissionAllMiddleboxes() {
  for (auto &mboxes : deployed_mboxes) mboxes.clear();
}

void ReleaseBandwidth() {
  for (auto &adj_list : graph) {
    for (auto &endpoint : adj_list) {
      endpoint.residual_bandwidth = endpoint.bandwidth;
    }
  }
}

void ReleaseCPU() {
  for (auto &n : nodes) {
    n.residual_cores = n.num_cores;
  }
}

inline void ReleaseAllResources() {
  DecommissionAllMiddleboxes();
  ReleaseCPU();
  ReleaseBandwidth();
}

inline void ReducePathResidualBandwidth(int source, int destination,
                                        int bandwidth) {
  std::pair<int, int> cache_index(source, destination);
  std::vector<int> *path_ptr = nullptr;
  if (path_cache[cache_index]) {
    path_ptr = path_cache[cache_index].get();
  } else {
    auto path = ComputeShortestPath(source, destination);
    path_cache[cache_index] = std::move(path);
    path_ptr = path_cache[cache_index].get();
  }
  for (int i = 0; i < static_cast<int>(path_ptr->size()) - 1; ++i) {
    ReduceEdgeResidualBandwidth(path_ptr->at(i), path_ptr->at(i + 1),
                                bandwidth);
  }
}

inline void ReduceNodeCapacity(int node, const middlebox &m_box) {
  nodes[node].residual_cores -= m_box.cpu_requirement;
}

int UsedMiddleboxIndex(int current_node, const middlebox &m_box,
                       const traffic_request &t_request) {
  for (int i = 0; i < deployed_mboxes[current_node].size(); ++i) {
    if (deployed_mboxes[current_node][i].m_box->middlebox_name ==
        m_box.middlebox_name) {
      if (deployed_mboxes[current_node][i].residual_capacity >=
          t_request.min_bandwidth) {
        return i;
      }
    }
  }
  return NIL;
}

void UpdateMiddleboxInstances(int current_node, const middlebox &m_box,
                              const traffic_request &t_request) {
  int used_middlebox_index = UsedMiddleboxIndex(current_node, m_box, t_request);
  if (used_middlebox_index != NIL) {
    deployed_mboxes[current_node][used_middlebox_index].residual_capacity -=
        t_request.min_bandwidth;
  } else {
    deployed_mboxes[current_node].emplace_back(
        &m_box, m_box.processing_capacity - t_request.min_bandwidth);
    ReduceNodeCapacity(current_node, m_box);
  }
}

void UpdateResources(const std::vector<int> *traffic_sequence,
                     const traffic_request &t_request) {
  for (int i = 0; i < static_cast<int>(traffic_sequence->size()) - 1; ++i) {
    ReducePathResidualBandwidth(traffic_sequence->at(i),
                                traffic_sequence->at(i + 1),
                                t_request.min_bandwidth);
  }
  for (int i = 1; i < static_cast<int>(traffic_sequence->size()) - 1; ++i) {
    const middlebox &m_box = middleboxes[t_request.middlebox_sequence[i - 1]];
    UpdateMiddleboxInstances(traffic_sequence->at(i), m_box, t_request);
  }
}

inline int IsResourceAvailable(int prev_node, int current_node,
                               const resource &resource_vector,
                               const middlebox &m_box,
                               const traffic_request &t_request) {
  DEBUG(
      "[IsResourceAvailable(%d, %d)] res_bw = %d, req_bw = %d, res_cores = %d,"
      "req_cores = %d\n",
      prev_node, current_node,
      GetPathResidualBandwidth(prev_node, current_node),
      t_request.min_bandwidth, resource_vector.cpu_cores[current_node],
      m_box.cpu_requirement);
  if ((GetPathResidualBandwidth(prev_node, current_node) >=
       t_request.min_bandwidth)) {
    // Check if we can use existing middlebox of the same type.
    if (UsedMiddleboxIndex(current_node, m_box, t_request) != NIL) {
      return 1;
    }
    // If we cannot use existing ones, then we need to instantiate new one.
    if (resource_vector.cpu_cores[current_node] >= m_box.cpu_requirement) {
      return 1;
    }
  }
  return 0;
}

inline double GetSLAViolationCost(int prev_node, int current_node,
                                  const traffic_request &t_request,
                                  const middlebox &m_box) {
  const int kNumSegments = t_request.middlebox_sequence.size() + 1;
  const double kPerSegmentLatencyBound =
      (1.0 * t_request.max_delay) / kNumSegments;
  if (shortest_path[prev_node][current_node] + m_box.processing_delay >
      kPerSegmentLatencyBound)
    return (shortest_path[prev_node][current_node] + m_box.processing_delay -
            kPerSegmentLatencyBound) *
           t_request.delay_penalty;
  return 0.0;
}

double GetSLAViolationCost(int source, int destination, double max_delay,
                           double penalty) {
  if (shortest_path[source][destination] > max_delay) {
    return penalty * (shortest_path[source][destination] - max_delay);
  }
  return 0;
}

inline double GetTransitCost(int prev_node, int current_node,
                             const traffic_request &t_request) {
  std::vector<int> *path_ptr = nullptr;
  std::pair<int, int> cache_index(prev_node, current_node);
  if (path_cache[cache_index]) {
    path_ptr = path_cache[cache_index].get();
  } else {
    path_cache[cache_index] = ComputeShortestPath(prev_node, current_node);
    path_ptr = path_cache[cache_index].get();
  }
  if (path_ptr) {
    int path_length = path_ptr->size() - 1;
    return (1.0 / 1000.0) * path_length * per_bit_transit_cost *
           t_request.min_bandwidth * t_request.duration;
  }
  return INF;
}

double GetServerEnergyConsumption(int num_cores_used) {
  int full_servers_used = num_cores_used / NUM_CORES_PER_SERVER;
  double energy_consumed =
      static_cast<double>(full_servers_used * SERVER_PEAK_ENERGY);
  int residual_cores = num_cores_used % NUM_CORES_PER_SERVER;
  energy_consumed += POWER_CONSUMPTION_ONE_SERVER(residual_cores);
  return energy_consumed;
}

inline double GetEnergyCost(int current_node, const middlebox &m_box,
                            const resource &resource_vector,
                            const traffic_request &t_request) {
  if (UsedMiddleboxIndex(current_node, m_box, t_request) != NIL) {
    return 0;
  }
  int previously_used_cores =
      nodes[current_node].num_cores - resource_vector.cpu_cores[current_node];
  int currently_used_cores = previously_used_cores + m_box.cpu_requirement;
  double duration_hours =
      static_cast<double>(t_request.duration) / (60.0 * 60.0);
  double previous_cost = GetServerEnergyConsumption(previously_used_cores) *
                         duration_hours * PER_UNIT_ENERGY_PRICE;
  double current_cost = GetServerEnergyConsumption(currently_used_cores) *
                        duration_hours * PER_UNIT_ENERGY_PRICE;
  return current_cost - previous_cost;
}

inline double GetDeploymentCost(int current_node, const middlebox &m_box,
                                const traffic_request &t_request) {
  // If we can use existing middlebox then there is no deployment cost.
  if (UsedMiddleboxIndex(current_node, m_box, t_request) != NIL) {
    return 0.0;
  }
  return m_box.deployment_cost;
}

double GetCost(int prev_node, int current_node, const resource &resource_vector,
               const middlebox &m_box, const traffic_request &t_request) {
  double deployment_cost = GetDeploymentCost(current_node, m_box, t_request);
  double energy_cost =
      GetEnergyCost(current_node, m_box, resource_vector, t_request);
  double transit_cost = GetTransitCost(prev_node, current_node, t_request);
  transit_cost +=
      GetTransitCost(current_node, t_request.destination, t_request);
  double sla_violation_cost =
      GetSLAViolationCost(prev_node, current_node, t_request, m_box);
  DEBUG("dep_cost = %lf, en_cost = %lf, tr_cost = %lf,"
        "sla_cost = %lf\n",
        deployment_cost, energy_cost, transit_cost, sla_violation_cost);
  return deployment_cost + energy_cost + transit_cost + sla_violation_cost;
}
inline int GetLatency(int source, int destination) {
  for (edge_endpoint endpoint : graph[source]) {
    if (endpoint.u->node_id == destination) return endpoint.delay;
  }
  return NIL;
}

void ComputeSolutionCosts(const std::vector<std::vector<int>> &solutions) {
  int current_time = traffic_requests[0].arrival_time;
  for (int i = 0; i < solutions.size(); ++i) {
    if (current_time != traffic_requests[i].arrival_time) {
      RefreshServerStats(current_time);
      current_time = traffic_requests[i].arrival_time;
      ReleaseAllResources();
    }
    auto &current_solution = solutions[i];
    double d_cost = 0.0, e_cost = 0.0, t_cost = 0.0, s_cost = 0.0;
    const int kLastIndex = static_cast<int>(current_solution.size()) - 1;
    double total_delay = 0.0;
    int embedded_path_length = 0;
    resource resource_vector;
    for (auto &node : nodes)
      resource_vector.cpu_cores.push_back(node.residual_cores);
    for (int kk = 1; kk < current_solution.size(); ++kk) {
      auto &m_box = middleboxes[traffic_requests[i].middlebox_sequence[kk - 1]];
      int current_node = current_solution[kk];
      int prev_node = current_solution[kk - 1];

      // Deployment Cost.
      if (kk != kLastIndex) {
        d_cost += GetDeploymentCost(current_node, m_box, traffic_requests[i]);
      }

      // Energy Cost.
      if (kk != kLastIndex) {
        e_cost += GetEnergyCost(current_node, m_box, resource_vector,
                                traffic_requests[i]);
      }

      // Transit Cost.
      t_cost += GetTransitCost(prev_node, current_node, traffic_requests[i]);

      // Update the resource vector with any new middleboxes.
      if (kk != kLastIndex &&
          UsedMiddleboxIndex(current_node, m_box, traffic_requests[i]) == NIL) {
        resource_vector.cpu_cores[current_node] -= m_box.cpu_requirement;
      }

      // Compute total delay for SLA violation cost.
      total_delay += shortest_path[prev_node][current_node];
      if (kk != 0 && kk != kLastIndex) {
        total_delay +=
            middleboxes[traffic_requests[i].middlebox_sequence[kk - 1]]
                .processing_delay;
      }
    }

    // SLA violation cost.
    double sla_cost = 0.0;
    if (total_delay > traffic_requests[i].max_delay) {
      sla_cost = (total_delay - traffic_requests[i].max_delay) *
                 traffic_requests[i].delay_penalty;
    }

    deployment_costs.push_back(d_cost);
    energy_costs.push_back(e_cost);
    transit_costs.push_back(t_cost);
    sla_costs.push_back(sla_cost);
    total_costs.push_back(d_cost + e_cost + t_cost + sla_cost);
    UpdateResources(&current_solution, traffic_requests[i]);
    RefreshServerStats(current_time);
  }
}

double GetSolutionStretch(const std::vector<int> &result) {
  int embedded_path_length = 0;
  const int kSequenceLength = result.size();
  int kSource = result[0];
  int kDestination = result[kSequenceLength - 1];
  int shortest_path_length =
      ComputeShortestPath(kSource, kDestination)->size() - 1;
  for (int i = 0; i < kSequenceLength - 1; ++i) {
    embedded_path_length +=
        ComputeShortestPath(result[i], result[i + 1])->size() - 1;
  }
  return static_cast<double>(embedded_path_length) /
         static_cast<double>(shortest_path_length);
}

void ComputeAllStretches(const std::vector<std::vector<int>> &solutions) {
  for (auto &current_solution : solutions) {
    stretches.push_back(GetSolutionStretch(current_solution));
  }
}

void ProcessCostLogs(const std::string& output_file_prefix) {
  const std::string kCostTsFileName = output_file_prefix + ".cost.ts";
  FILE* cost_ts_file = fopen(kCostTsFileName.c_str(), "w");
  std::vector<double> cost_ts_data;
  // Log time series data for cost.
  int current_time = traffic_requests[0].arrival_time;
  double current_cost = 0.0;
  double current_d_cost = 0.0;
  double current_e_cost = 0.0;
  double current_t_cost = 0.0;
  double current_sla_cost = 0.0;
  for (int i = 0; i < traffic_requests.size(); ++i) {
    if (current_time != traffic_requests[i].arrival_time) {
      cost_ts_data.push_back(current_cost);
      fprintf(cost_ts_file, "%d %.4lf %.4lf %.4lf %.4lf %.4lf\n", 
              current_time, current_cost, current_d_cost, current_e_cost,
              current_t_cost, current_sla_cost);
      current_time = traffic_requests[i].arrival_time;
      current_cost = current_d_cost = current_e_cost = current_t_cost =
          current_sla_cost = 0.0;
    }
    current_cost += total_costs[i];
    current_d_cost += deployment_costs[i];
    current_e_cost += energy_costs[i];
    current_t_cost += transit_costs[i];
    current_sla_cost += sla_costs[i];
  }
  cost_ts_data.push_back(current_cost);
  fprintf(cost_ts_file, "%d %.4lf %.4lf %.4lf %.4lf %.4lf\n", 
          current_time, current_cost, current_d_cost, current_e_cost,
          current_t_cost, current_sla_cost);
  fclose(cost_ts_file);

  // Log mean, 5th, and 95th percentile of the total cost.
  const std::string kCostSummaryFileName = output_file_prefix + ".cost.summary";
  double mean_cost = GetMean(cost_ts_data);
  double fifth_percentile_cost = GetNthPercentile(cost_ts_data, 5);
  double ninety_fifth_percentile_cost = GetNthPercentile(cost_ts_data, 95);
  FILE* cost_summary_file = fopen(kCostSummaryFileName.c_str(), "w");
  fprintf(cost_summary_file, "%.3lf %.3lf %.3lf\n", mean_cost,
          fifth_percentile_cost, ninety_fifth_percentile_cost);
  fclose(cost_summary_file);
}

void ProcessStretchLogs(const std::string& output_file_prefix) {
  const std::string kStretchFileName = output_file_prefix + ".stretch";
  FILE* stretch_file = fopen(kStretchFileName.c_str(), "w");
  printf("getting cdf\n");
  std::vector<std::pair<double, double>> cdf = GetCDF(stretches);
  printf("got cdf\n");
  for (int i = 0; i < cdf.size(); ++i) {
    fprintf(stretch_file, "%.3lf %.3lf\n", cdf[i].first, cdf[i].second);
  }
  fclose(stretch_file);
}

void ProcessStats(solution_statistics &s,
                  const std::string &output_file_prefix) {
  const std::string kCostTsFileName = output_file_prefix + ".cost.ts";
  const std::string kCostSummaryFileName = output_file_prefix + ".cost.summary";
  const std::string kUtilTsFileName = output_file_prefix + ".util.ts";
  const std::string kFragmentationFileName = output_file_prefix + ".frag.ts";
  // Process cost data.
  FILE *cost_ts_file = fopen(kCostTsFileName.c_str(), "w");
  int current_time = 0;
  std::vector<double> cost_data;
  // std::vector<double> global_cost_data;
  double current_cost = 0.0;
  s.t_stats.push_back(traffic_statistics(INF, INF));

  // First write the time series data to file. For each time instance write the
  // sum of costs from all traffic.
  for (auto &t_stats : s.t_stats) {
    if (current_time != t_stats.arrival_time) {
      cost_data.push_back(current_cost);
      fprintf(cost_ts_file, "%d %.3lf\n", current_time, current_cost);
      current_time = t_stats.arrival_time;
      current_cost = 0.0;
    }
    current_cost += t_stats.cost;
  }
  fclose(cost_ts_file);
  s.t_stats.pop_back();

  // Write the mean, 5th, and 95th percentile of the cost to file.
  double mean_cost = GetMean(cost_data);
  double fifth_percentile_cost = GetNthPercentile(cost_data, 5);
  double ninety_fifth_percentile_cost = GetNthPercentile(cost_data, 95);
  FILE *cost_summary_file = fopen(kCostSummaryFileName.c_str(), "w");
  fprintf(cost_summary_file, "%.3lf %.3lf %.3lf\n", mean_cost,
          fifth_percentile_cost, ninety_fifth_percentile_cost);
  fclose(cost_summary_file);

  // Process utilization data. Also derive fragmentation data from utilization
  // data: fragmentation = 1 - utilization.
  FILE *util_ts_file = fopen(kUtilTsFileName.c_str(), "w");
  FILE *fragmentation_ts_file = fopen(kFragmentationFileName.c_str(), "w");
  current_time = 0;
  std::vector<double> util_data;
  s.server_stats.emplace_back(INF, NIL, INF);
  DEBUG("bingo\n");
  std::vector<std::vector<double>> per_server_util;
  per_server_util.resize(graph.size());
  for (auto &server_stat : s.server_stats) {
    if (server_stat.server_id != NIL) {
      per_server_util[server_stat.server_id].push_back(server_stat.utilization);
    }
    if (current_time != server_stat.timestamp) {
      double mean_util = GetMean(util_data);
      double fifth_percentile_util = GetNthPercentile(util_data, 5);
      double ninety_fifth_percentile_util = GetNthPercentile(util_data, 95);
      fprintf(util_ts_file, "%d %.3lf %.3lf %.3lf\n", current_time, mean_util,
              fifth_percentile_util, ninety_fifth_percentile_util);
      double mean_fragmentation = 1 - mean_util;
      double fifth_percentile_fragmentation = 1 - fifth_percentile_util;
      double ninety_fifth_percentile_fragmentation =
          1 - ninety_fifth_percentile_util;
      fprintf(fragmentation_ts_file, "%d %.3lf %.3lf %.3lf\n", current_time,
              mean_fragmentation, fifth_percentile_fragmentation,
              ninety_fifth_percentile_fragmentation);
      current_time = server_stat.timestamp;
      util_data.clear();
    }
    util_data.push_back(server_stat.utilization);
  }
  fclose(util_ts_file);
  fclose(fragmentation_ts_file);

  // Process per server utilization data.
  const std::string kPerServerUtilFileName =
      output_file_prefix + ".log.per_server_util";
  int util_cdf[101] = {0};
  FILE *per_server_util_file = fopen(kPerServerUtilFileName.c_str(), "w");
  for (int i = 0; i < per_server_util.size(); ++i) {
    if (per_server_util[i].size() > 0) {
      double mean_util = GetMean(per_server_util[i]);
      double fifth_percentile_util = GetNthPercentile(per_server_util[i], 5);
      double ninety_fifth_percentile_util =
          GetNthPercentile(per_server_util[i], 95);
      fprintf(per_server_util_file, "Server-%d %.3f %.3f %.3f\n", i, mean_util,
              fifth_percentile_util, ninety_fifth_percentile_util);
      util_cdf[static_cast<int>(mean_util * 100.0)]++;
    }
  }
  fclose(per_server_util_file);

  // Process CDF of mean server utilization.
  const std::string kServerUtilCdfFile = output_file_prefix + ".log.util.cdf";
  FILE *server_util_cdf_file = fopen(kServerUtilCdfFile.c_str(), "w");
  int frequency_sum = 0;
  for (int i = 1; i <= 100; ++i) {
    util_cdf[i] += util_cdf[i - 1];
    frequency_sum = util_cdf[i];
  }
  for (int i = 1; i <= 100; ++i) {
    double util = static_cast<double>(i) / 100.0;
    double cdf =
        static_cast<double>(util_cdf[i]) / static_cast<double>(frequency_sum);
    fprintf(server_util_cdf_file, "%.3lf %.3lf\n", util, cdf);
  }
  fclose(server_util_cdf_file);

  // Percentage of middleboxes deployed withing k distance of the ingress/egress
  const std::string kIngressK = output_file_prefix + ".ingress.k";
  const std::string kEgressK = output_file_prefix + ".egress.k";
  FILE *ingress_k_cdf_file = fopen(kIngressK.c_str(), "w");
  FILE *egress_k_cdf_file = fopen(kEgressK.c_str(), "w");
  std::vector<int> ingress_k(graph.size() + 1, 0);
  std::vector<int> egress_k(graph.size() + 1, 0);
  int num_elements = 0;
  for (auto &t_stats : s.t_stats) {
    num_elements += t_stats.ingress_hops.size();
    for (auto &hops : t_stats.ingress_hops) {
      ingress_k[hops]++;
    }
    for (auto &hops : t_stats.egress_hops) {
      egress_k[hops]++;
    }
  }
  for (int i = 1; i <= graph.size(); ++i) {
    ingress_k[i] += ingress_k[i - 1];
    egress_k[i] += egress_k[i - 1];
  }
  for (int i = 0; i <= graph.size(); ++i) {
    double cdf =
        static_cast<double>(ingress_k[i]) / static_cast<double>(num_elements);
    fprintf(ingress_k_cdf_file, "%d %.3lf\n", i, cdf);
    cdf = static_cast<double>(egress_k[i]) / static_cast<double>(num_elements);
    fprintf(egress_k_cdf_file, "%d %.3lf\n", i, cdf);
  }
  fclose(ingress_k_cdf_file);
  fclose(egress_k_cdf_file);

  // Process stretch data.
  const std::string kStretchFileName = output_file_prefix + ".stretch.cdf";
  FILE *stretch_file = fopen(kStretchFileName.c_str(), "w");
  const int kNumTraffic = s.t_stats.size();
  double max_stretch = -1;
  for (auto &t_stats : s.t_stats) {
    if (max_stretch < t_stats.stretch) max_stretch = t_stats.stretch;
  }
  std::vector<int> stretch_cdf(static_cast<int>(max_stretch * 100.0), 0.0);
  for (auto &t_stats : s.t_stats) {
    ++stretch_cdf[static_cast<int>(t_stats.stretch * 100.0)];
  }
  for (int i = 1; i < stretch_cdf.size(); ++i) {
    stretch_cdf[i] += stretch_cdf[i - 1];
    double I = static_cast<double>(i) / 100.0;
    double cdf =
        static_cast<double>(stretch_cdf[i]) / static_cast<double>(kNumTraffic);
    fprintf(stretch_file, "%.3lf %.3lf\n", I, cdf);
  }
  fclose(stretch_file);
}

#endif  // MIDDLEBOX_PLACEMENT_SRC_UTIL_H_
