#ifndef MIDDLEBOX_PLACEMENT_SRC_VITERBI_H_
#define MIDDLEBOX_PLACEMENT_SRC_VITERBI_H_

#include "datastructure.h"
#include "util.h"
#include <algorithm>


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
                                  const traffic_request &t_request) {
  const int kNumSegments = t_request.middlebox_sequence.size() + 1;
  const double kPerSegmentLatencyBound =
      (1.0 * t_request.max_delay) / kNumSegments;
  if (shortest_path[prev_node][current_node] > kPerSegmentLatencyBound)
    return (shortest_path[prev_node][current_node] - kPerSegmentLatencyBound) *
           t_request.delay_penalty;
  return 0.0;
}

double GetSLAViolationCost(int source, int destination, double max_delay, double penalty) {
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
  double sla_violation_cost =
      GetSLAViolationCost(prev_node, current_node, t_request);
  DEBUG("dep_cost = %lf, en_cost = %lf, tr_cost = %lf,"
        "sla_cost = %lf\n",
        deployment_cost, energy_cost, transit_cost, sla_violation_cost);
  return deployment_cost + energy_cost + transit_cost + sla_violation_cost;
}

void ViterbiInit() {
  for (int i = 0; i < MAXN; ++i) {
    for (int j = 0; j < MAXN; ++j) {
      cost[i][j] = INF;
      pre[i][j] = NIL;
    }
  }
}

std::unique_ptr<std::vector<int> > ViterbiCompute(
    const traffic_request &t_request) {
  ViterbiInit();
  int stage = 0, node = NIL;
  const static int kNumNodes = graph.size();
  const int kNumStages = t_request.middlebox_sequence.size();
  std::vector<resource> current_vector, previous_vector;
  current_vector.resize(kNumNodes);
  previous_vector.resize(kNumNodes);

  for (int i = 0; i < kNumNodes; ++i) {
    for (int j = 0; j < kNumNodes; ++j) {
      current_vector[i].cpu_cores.push_back(nodes[j].residual_cores);
    }
  }
  for (node = 0; node < kNumNodes; ++node) {
    const middlebox &m_box = middleboxes[t_request.middlebox_sequence[0]];
    if (IsResourceAvailable(t_request.source, node, current_vector[node], m_box,
                            t_request)) {
      cost[stage][node] = GetCost(t_request.source, node, current_vector[node],
                                  m_box, t_request);
      current_vector[node].cpu_cores[node] -= m_box.cpu_requirement;
      DEBUG("[First stage] cost[stage][node] = %lf\n", cost[stage][node]);
    }
  }
  // TODO(shihab): Handle repeated middleboxes.
  // TODO(shihab): Handle middlebox reuse.
  for (stage = 1; stage < kNumStages; ++stage) {
    const middlebox &m_box = middleboxes[t_request.middlebox_sequence[stage]];
    previous_vector = current_vector;
    DEBUG("[stage = %d] Placing middlebox = %s\n", stage,
          m_box.middlebox_name.c_str());
    for (int current_node = 0; current_node < kNumNodes; ++current_node) {
      int min_index = NIL;
      for (int prev_node = 0; prev_node < kNumNodes; ++prev_node) {
        if (IsResourceAvailable(prev_node, current_node,
                                previous_vector[prev_node], m_box, t_request)) {
          double transition_cost =
              cost[stage - 1][prev_node] +
              GetCost(prev_node, current_node, previous_vector[prev_node],
                      m_box, t_request);
          DEBUG("[stage = %d, middlebox = %s, prev_node = %d, tr_cost = "
                "%lf]\n",
                stage, m_box.middlebox_name.c_str(), prev_node,
                transition_cost);
          if (cost[stage][current_node] > transition_cost) {
            cost[stage][current_node] = transition_cost;
            pre[stage][current_node] = prev_node;
            min_index = prev_node;
          }
        }
      }
      DEBUG("[stage = %d, min_index = %d]\n", stage, min_index);
      if (min_index != NIL) {
        DEBUG("Current node = %d, min_index = %d\n", current_node, min_index);
        current_vector[current_node].cpu_cores =
            previous_vector[min_index].cpu_cores;
        bool new_middlebox_deployed = true;
        for (middlebox_instance &mbox_instance :
             deployed_mboxes[current_node]) {
          if (mbox_instance.m_box->middlebox_name == m_box.middlebox_name &&
              mbox_instance.residual_capacity >= t_request.min_bandwidth) {
            new_middlebox_deployed = false;
            break;
          }
        }
        if (new_middlebox_deployed) {
          current_vector[current_node].cpu_cores[current_node] -=
              m_box.cpu_requirement;
        }
      } else {
        current_vector[current_node].cpu_cores.clear();
      }
    }
  }

  // Find the solution sequence
  double min_cost = INF;
  int min_index = NIL;
  for (int cur_node = 0; cur_node < kNumNodes; ++cur_node) {
    double transition_cost =
        cost[kNumStages - 1][cur_node] +
        GetTransitCost(cur_node, t_request.destination, t_request) +
        GetSLAViolationCost(cur_node, t_request.destination, t_request);
    if (min_cost > transition_cost) {
      min_cost = transition_cost;
      min_index = cur_node;
    }
  }
  if (min_index < 0) {
    ++stats.num_rejected;
    return std::unique_ptr<std::vector<int> >(new std::vector<int>());
  }
  // Update statistics.
  ++stats.num_accepted;
  stats.t_stats.emplace_back(t_request.arrival_time, min_cost);
  std::unique_ptr<std::vector<int> > return_vector(new std::vector<int>());
  int current_node = min_index;
  for (stage = kNumStages - 1; stage >= 0; --stage) {
    return_vector->push_back(current_node);
    current_node = pre[stage][current_node];
  }
  DEBUG("Computed vector size = %d\n", return_vector->size());
  return_vector->push_back(t_request.source);
  std::reverse(return_vector->begin(), return_vector->end());
  return_vector->push_back(t_request.destination);
  for (int i = 1; i < static_cast<int>(return_vector->size()) - 1; ++i) {
    stats.t_stats.back().ingress_hops.push_back(
        ComputeShortestPath(t_request.source, return_vector->at(i))->size() -
        1);
    stats.t_stats.back().egress_hops
        .push_back(ComputeShortestPath(return_vector->at(i),
                                       t_request.destination)->size() -
                   1);
  }
  return std::move(return_vector);
}

void UpdateResources(std::vector<int> *traffic_sequence,
                     const traffic_request &t_request) {
  int shortest_path_length =
      ComputeShortestPath(t_request.source, t_request.destination)->size() - 1;
  int embedded_path_length = 0;
  double total_delay = 0.0;
  for (int i = 0; i < static_cast<int>(traffic_sequence->size()) - 1; ++i) {
    ReducePathResidualBandwidth(traffic_sequence->at(i),
                                traffic_sequence->at(i + 1),
                                t_request.min_bandwidth);
    embedded_path_length +=
        ComputeShortestPath(traffic_sequence->at(i),
                            traffic_sequence->at(i + 1))->size() -
        1;
    total_delay += shortest_path[traffic_sequence->at(i)][traffic_sequence->at(i + 1)];
  }
  double sla_penalty = 0.0;
  if (total_delay > t_request.max_delay) {
    sla_penalty = (total_delay - t_request.max_delay) * t_request.delay_penalty;
  }
  sla_costs.push_back(sla_penalty);
  stats.t_stats.back().stretch = static_cast<double>(embedded_path_length) /
                                 static_cast<double>(shortest_path_length);
  for (int i = 1; i < static_cast<int>(traffic_sequence->size()) - 1; ++i) {
    // for (int i = 0; i < t_request.middlebox_sequence.size(); ++i) {
    //  DEBUG("i = %d, t_request.middlebox_sequence.size() = %d,"
    //  "traffic_sequence->size() = %d, t_request.middlebox_sequence[i] = %d",
    //  i, t_request.middlebox_sequence.size(), traffic_sequence->size(),
    //  t_request.middlebox_sequence[i]);
    const middlebox &m_box = middleboxes[t_request.middlebox_sequence[i - 1]];
    UpdateMiddleboxInstances(traffic_sequence->at(i), m_box, t_request);
    // ReduceNodeCapacity(traffic_sequence->at(i), m_box);
  }
}
#endif  //  MIDDLEBOX_PLACEMENT_SRC_VITERBI_H_
