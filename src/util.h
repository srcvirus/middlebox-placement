#ifndef MIDDLEBOX_PLACEMENT_SRC_UTIL_H_
#define MIDDLEBOX_PLACEMENT_SRC_UTIL_H_
#include "datastructure.h"

#include <algorithm>
#include <assert.h>
#include <set>
#include <stack>
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

int GetBandwidthUsage(const std::vector<int>& traffic_sequence,
                      const traffic_request& t_request)
{
  int bandwidth_usage = 0;
  for (int i = 0; i < traffic_sequence.size() - 1; ++i) {
    bandwidth_usage += (t_request.min_bandwidth * 
                         ComputeShortestPath(traffic_sequence[i],
                                             traffic_sequence[i + 1])->size() 
                                               - 1);
  }
  return bandwidth_usage;
}

long GetTotalNetworkBandwidth() {
  long total_bandwidth = 0;
  for (int i = 0; i < graph.size(); ++i) {
    for (auto& endpoint : graph[i]) {
      total_bandwidth += endpoint.bandwidth;
    }
  }
  return total_bandwidth;
}

void ComputeSolutionCosts(const std::vector<std::vector<int>> &solutions) {
  int current_time = traffic_requests[0].arrival_time;
  for (int i = 0; i < solutions.size(); ++i) {
    if (current_time != traffic_requests[i].arrival_time) {
      RefreshServerStats(current_time);
      current_time = traffic_requests[i].arrival_time;
      int n_deployed = 0;
      for (int j = 0; j < deployed_mboxes.size(); ++j)
        n_deployed += deployed_mboxes.size();
      mbox_count.push_back(n_deployed);
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

void CplexComputeAllStretches(
  const std::vector<std::vector<int>>& solution_paths) {
  for (auto& path : solution_paths) {
    int embedded_path_length = path.size() - 1;
    int source = path[0];
    int destination = path[embedded_path_length];
    int shortest_path_length = ComputeShortestPath(
                                 source, destination)->size() - 1;
    double stretch = static_cast<double>(embedded_path_length) / 
                      static_cast<double>(shortest_path_length);
    stretches.push_back(stretch);                      
  }
}

void ComputeNetworkUtilization(
  const std::vector<std::vector<int>>& solutions) {
  const long kNetworkCapacity = GetTotalNetworkBandwidth();
  for(int i = 0; i < traffic_requests.size(); ++i) {
    long bandwidth_usage = GetBandwidthUsage(solutions[i], traffic_requests[i]);
    net_util.push_back(static_cast<double>(bandwidth_usage) /
                         static_cast<double>(kNetworkCapacity));
  }
}

void CplexComputeNetworkUtilization(
  const std::vector<std::vector<int>>& solution_paths) {
  int i = 0;
  const long kNetworkCapacity = GetTotalNetworkBandwidth();
  for (auto& t_request : traffic_requests) {
    long bandwidth_usage = (solution_paths[i++].size() - 1) *
                             t_request.min_bandwidth;
    net_util.push_back(static_cast<double>(bandwidth_usage) /
                         static_cast<double>(kNetworkCapacity));
  }
}

void ComputeKHops(
  const std::vector<std::vector<int>>& solutions) {
 ingress_k.resize(solutions.size());
 egress_k.resize(solutions.size());
 for (int i = 0; i < solutions.size(); ++i) {
   int ingress = solutions[i].front();
   int egress = solutions[i].back();
   int ihops = 0, ehops = 0;
   for (int j = 1; j < solutions[i].size() - 1; ++j) {
     ihops += ComputeShortestPath(solutions[i][j - 1], solutions[i][j])->size() - 1;
     ingress_k[i].push_back(ihops);
   }
   for (int j = solutions[i].size() - 2; j >= 1; --j) {
     ehops += ComputeShortestPath(solutions[i][j + 1], solutions[i][j])->size() - 1;
     egress_k[i].push_back(ehops);
   }
 }
}

void ComputeCloseness(
  const std::vector<std::vector<int>>& solutions) {
  sol_closeness.resize(solutions.size());
  int i = 0;
  for (auto& current_solution : solutions) {
    for (auto& element : current_solution) {
      sol_closeness[i].push_back(closeness[element]);
    }
    ++i;
  }
}

void CplexComputeKHops(
  const std::vector<std::vector<int>>& solutions,
  const std::vector<std::vector<int>>& solution_paths) {
  ingress_k.resize(solutions.size());
  egress_k.resize(solutions.size());
  for (int i = 0; i < solutions.size(); ++i) {
    int ingress = solution_paths[i].front();
    int egress = solution_paths[i].back();
    int ihops = 0, ehops = 0;
    int kk = 0;
    for (int j = 1; j < solutions[i].size() - 1; ++j) {
      for ( ; kk < solution_paths[i].size() - 1; ++kk) {
        if (solution_paths[i][kk] == solutions[i][j]) break;
      }
      ihops = kk;
      ehops = solution_paths[i].size() - kk - 1;
      ingress_k[i].push_back(ihops);
      egress_k[i].push_back(ehops);
    }
  }
}

void ComputeServicePoints(const std::vector<std::vector<int>>& solutions) {
  for (auto& current_solution : solutions) {
    std::set<int> S;
    for (int i = 1; i < current_solution.size() - 1; ++i) {
      S.insert(current_solution[i]);
    }
    num_service_points.push_back(S.size());
  }
}

void ProcessServicePointLogs(const std::string& output_file_prefix) {
  const std::string kServicePointLogFile = output_file_prefix +
                                              ".service_points";
  FILE* service_point_log = fopen(kServicePointLogFile.c_str(), "w");
  std::vector <std::pair<int, double>> cdf = GetCDF(num_service_points);
  for (auto& cdf_element : cdf) {
    fprintf(service_point_log, "%d %lf\n", cdf_element.first,
                                           cdf_element.second);
  }
  fclose(service_point_log);
}

void ProcessMboxRatio(const std::string& output_file_prefix) {
  int traffic_count = 0;
  int current_time = traffic_requests[0].arrival_time;
  const std::string kMboxRatioFileName = output_file_prefix + ".mbox.ratio";
  FILE* mbox_ratio_file = fopen(kMboxRatioFileName.c_str(), "w");
  const int kMboxSeqSize = traffic_requests[0].middlebox_sequence.size();
  for (int i = 0; i < traffic_requests.size(); ++i) {
    if (current_time != traffic_requests[i].arrival_time) {
      int nmbox = mbox_count.front();
      mbox_count.pop_front();
      fprintf(mbox_ratio_file, "%d %d %lu\n", current_time, nmbox,
               traffic_requests[i].middlebox_sequence.size() * traffic_count);
      traffic_count = 0;
      current_time = traffic_requests[i].arrival_time;
    }
    ++traffic_count;
  }
  if (!mbox_count.empty()) {
    fprintf(mbox_ratio_file, "%d %d %d\n", current_time, mbox_count.front(),
             kMboxSeqSize * traffic_count);
  }
  fclose(mbox_ratio_file);
}

void ProcessKHopsLogs(const std::string& output_file_prefix) {
  std::vector<int> ihops, ehops;
  const std::string kIngressKHopsFileName = output_file_prefix +
                                              ".ingress_k.cdf";
  const std::string kEgressKHopsFileName = output_file_prefix +
                                             ".egress_k.cdf";
  FILE* ingress_k_file = fopen(kIngressKHopsFileName.c_str(), "w");
  FILE* egress_k_file = fopen(kEgressKHopsFileName.c_str(), "w");
  for (int i = 0; i < traffic_requests.size(); ++i) {
    for (auto& elem : ingress_k[i]) ihops.push_back(elem);
    for (auto& elem : egress_k[i]) ehops.push_back(elem);
  }
  std::vector<std::pair<int,double>> ingress_k_cdf = GetCDF(ihops);
  std::vector<std::pair<int,double>> egress_k_cdf = GetCDF(ehops);
  for (auto& cdf : ingress_k_cdf) {
    fprintf(ingress_k_file, "%d %lf\n", cdf.first, cdf.second);
  }
  for (auto& cdf : egress_k_cdf) {
    fprintf(egress_k_file, "%d %lf\n", cdf.first, cdf.second);
  }
  fclose(ingress_k_file);
  fclose(egress_k_file);
}

void ProcessNetUtilizationLogs(const std::string& output_file_prefix) {
  // Write time series data for utilization.
  const std::string kNetUtilTsFileName = output_file_prefix + ".netutil.ts";
  FILE* netutil_ts_file = fopen(kNetUtilTsFileName.c_str(), "w");
  int current_time = traffic_requests[0].arrival_time;
  double current_util = 0.0;
  std::vector<double> netutil_ts_data;
  for (int i = 0; i < traffic_requests.size(); ++i) {
    if (current_time != traffic_requests[i].arrival_time) {
      netutil_ts_data.push_back(current_util);
      fprintf(netutil_ts_file, "%d %lf\n", current_time, current_util);
      current_time = traffic_requests[i].arrival_time;
      current_util = 0.0;
    }
    current_util += net_util[i];
  }
  netutil_ts_data.push_back(current_util);
  fprintf(netutil_ts_file, "%d %lf\n", current_time, current_util);
  fclose(netutil_ts_file);

  // Write mean, 5th and 95th percentile of this utilization data to file.
  double mean_util = GetMean(netutil_ts_data);
  double fifth_percentile_util = GetNthPercentile(netutil_ts_data, 5);
  double ninety_fifth_percentile_util = GetNthPercentile(netutil_ts_data, 95);
  const std::string kNetUtilSummaryFileName = output_file_prefix +
                                                ".netutil.summary";
  FILE* netutil_summary_file = fopen(kNetUtilSummaryFileName.c_str(), "w");
  fprintf(netutil_summary_file, "%lf %lf %lf\n", mean_util,
              fifth_percentile_util, ninety_fifth_percentile_util);
  fclose(netutil_summary_file);
}

void ProcessCostLogs(const std::string& output_file_prefix) {
  const std::string kCostTsFileName = output_file_prefix + ".cost.ts";
  const std::string kAllCostFileName = output_file_prefix + ".cost.all";
  FILE* cost_ts_file = fopen(kCostTsFileName.c_str(), "w");
  FILE* all_cost_file = fopen(kAllCostFileName.c_str(), "w");
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
      fprintf(cost_ts_file, "%d %lf %lf %lf %lf %lf\n", 
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
    fprintf(all_cost_file, "%d", current_time);
    for (int j = 0; j < results[i].size(); ++j) {
      fprintf(all_cost_file, " %d", results[i][j]);
    }
    fprintf(all_cost_file, " %lf %lf %lf %lf\n", energy_costs[i], transit_costs[i],
            sla_costs[i], total_costs[i]);
  }
  cost_ts_data.push_back(current_cost);
  fprintf(cost_ts_file, "%d %lf %lf %lf %lf %lf\n", 
          current_time, current_cost, current_d_cost, current_e_cost,
          current_t_cost, current_sla_cost);
  fclose(cost_ts_file);
  fclose(all_cost_file);
  // Log mean, 5th, and 95th percentile of the total cost.
  const std::string kCostSummaryFileName = output_file_prefix + ".cost.summary";
  double mean_cost = GetMean(cost_ts_data);
  double fifth_percentile_cost = GetNthPercentile(cost_ts_data, 5);
  double ninety_fifth_percentile_cost = GetNthPercentile(cost_ts_data, 95);
  FILE* cost_summary_file = fopen(kCostSummaryFileName.c_str(), "w");
  fprintf(cost_summary_file, "%lf %lf %lf\n", mean_cost,
          fifth_percentile_cost, ninety_fifth_percentile_cost);
  fclose(cost_summary_file);
}

void ProcessStretchLogs(const std::string& output_file_prefix) {
  const std::string kStretchFileName = output_file_prefix + ".stretch";
  FILE* stretch_file = fopen(kStretchFileName.c_str(), "w");
  std::vector<std::pair<double, double>> cdf = GetCDF(stretches);
  for (int i = 0; i < cdf.size(); ++i) {
    fprintf(stretch_file, "%lf %lf\n", cdf[i].first, cdf[i].second);
  }
  fclose(stretch_file);
}

void ProcessServerUtilizationLogs(const std::string& output_file_prefix) {
  // Process utilization data. Also derive fragmentation data from utilization
  // data: fragmentation = 1 - utilization.
  const std::string kUtilTsFileName = output_file_prefix + ".serverutil.ts";
  const std::string kFragmentationFileName = output_file_prefix +
                                               ".serverfrag.ts";
  FILE *util_ts_file = fopen(kUtilTsFileName.c_str(), "w");
  FILE *fragmentation_ts_file = fopen(kFragmentationFileName.c_str(), "w");
  int current_time = traffic_requests[0].arrival_time;
  std::vector<double> util_data;
  stats.server_stats.emplace_back(INF, NIL, INF);
  std::vector<std::vector<double>> per_server_util;
  per_server_util.resize(graph.size());
  for (auto &server_stat : stats.server_stats) {
    if (server_stat.server_id != NIL) {
      if (fabs(server_stat.utilization - 0.0) > EPS) { 
        per_server_util[server_stat.server_id].push_back(
                                                 server_stat.utilization);
      }
    }
    if (current_time != server_stat.timestamp) {
      double mean_util = GetMean(util_data);
      double fifth_percentile_util = GetNthPercentile(util_data, 5);
      double ninety_fifth_percentile_util = GetNthPercentile(util_data, 95);
      fprintf(util_ts_file, "%d %lf %lf %lf\n", current_time, mean_util,
              fifth_percentile_util, ninety_fifth_percentile_util);
      double mean_fragmentation = 1 - mean_util;
      double fifth_percentile_fragmentation = 1 - fifth_percentile_util;
      double ninety_fifth_percentile_fragmentation =
          1 - ninety_fifth_percentile_util;
      fprintf(fragmentation_ts_file, "%d %lf %lf %lf\n", current_time,
              mean_fragmentation, fifth_percentile_fragmentation,
              ninety_fifth_percentile_fragmentation);
      current_time = server_stat.timestamp;
      util_data.clear();
    }
    if (fabs(server_stat.utilization - 0.0) > EPS) {
      util_data.push_back(server_stat.utilization);
    }
  }
  fclose(util_ts_file);
  fclose(fragmentation_ts_file);

  // Process per server utilization data.
  const std::string kPerServerUtilFileName =
      output_file_prefix + ".per_server_util";
  FILE *per_server_util_file = fopen(kPerServerUtilFileName.c_str(), "w");
  std::vector<double> mean_util_data;
  for (int i = 0; i < per_server_util.size(); ++i) {
    if (per_server_util[i].size() > 0) {
      double mean_util = GetMean(per_server_util[i]);
      double fifth_percentile_util = GetNthPercentile(per_server_util[i], 5);
      double ninety_fifth_percentile_util =
          GetNthPercentile(per_server_util[i], 95);
      fprintf(per_server_util_file, "Server-%d %lf %lf %lf\n", i, mean_util,
              fifth_percentile_util, ninety_fifth_percentile_util);
      mean_util_data.push_back(mean_util);
    }
  }
  fclose(per_server_util_file);

  // Process CDF of mean server utilization.
  const std::string kServerUtilCdfFile = output_file_prefix + ".sutil.cdf";
  FILE *server_util_cdf_file = fopen(kServerUtilCdfFile.c_str(), "w");
  std::vector<std::pair<double, double>> util_cdf = GetCDF(mean_util_data);
  for (auto& cdf_data : util_cdf) {
    fprintf(server_util_cdf_file, "%lf %lf\n", cdf_data.first, cdf_data.second);
  }
  fclose(server_util_cdf_file);
}

void ProcessClosenessLogs(const std::string& output_file_prefix) {
  const std::string kClosenessLogFile = output_file_prefix + ".closeness.cdf";
  FILE* closeness_log = fopen(kClosenessLogFile.c_str(), "w");
  std::vector<double> data;
  for (int i = 0; i < sol_closeness.size(); ++i) {
    for (int j = 0; j < sol_closeness[i].size(); ++j) {
      data.push_back(sol_closeness[i][j]);
    }
  }
  std::vector<std::pair<double,double>> cdf = GetCDF(data);
  for (int i = 0; i < cdf.size(); ++i) {
    fprintf(closeness_log, "%lf %lf\n", cdf[i].first, cdf[i].second);
  }
  fclose(closeness_log);
}

std::vector<int> CplexComputePath(const std::vector<std::pair<int,int>>& edges,
                                  const std::vector<int> sequence) {
  int source = sequence.front();
  int destination = sequence.back();
  std::vector<std::vector<int>> adj;
  adj.resize(graph.size());
  std::vector<int> indeg(graph.size(), 0);
  std::vector<int> outdeg(graph.size(), 0);
  for (auto& edge : edges) {
    adj[edge.first].push_back(edge.second);
    ++outdeg[edge.first];
    ++indeg[edge.second];
  }
  for (int i = 0; i < graph.size(); ++i) {
    if ((indeg[i] + outdeg[i]) != 0) {
      if (i != source && i != destination) {
        assert(indeg[i] == outdeg[i]);
      } else {
        assert(abs(indeg[i] - outdeg[i]) == 1);
      }
    }
  }
  std::stack<int> s;
  std::vector<int> path;
  int current_node = source;
  while(true) {
    if (adj[current_node].empty()) {
      path.push_back(current_node);
      if (!s.empty()) {
        current_node = s.top();
        s.pop();
      } else break;
    } else {
      s.push(current_node);
      int neighbor = adj[current_node].back();
      adj[current_node].pop_back();
      current_node = neighbor;
    }
  }
  std::reverse(path.begin(), path.end());
  return path;
}

#endif  // MIDDLEBOX_PLACEMENT_SRC_UTIL_H_
