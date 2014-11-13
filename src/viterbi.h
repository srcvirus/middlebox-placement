#ifndef MIDDLEBOX_PLACEMENT_SRC_VITERBI_H_
#define MIDDLEBOX_PLACEMENT_SRC_VITERBI_H_

#include "datastructure.h"
#include "util.h"
#include <algorithm>

inline std::unique_ptr<std::vector<int> > ComputeShortestPath(int source,
                                                              int destination) {
  std::unique_ptr<std::vector<int> > path(new std::vector<int>());
  while (destination != NIL) {
    path->push_back(destination);
    destination = sp_pre[source][destination];
  }
  std::reverse(path->begin(), path->end());
  return std::move(path);
}

inline int GetLatency(int source, int destination) {
  for (edge_endpoint endpoint : graph[source]) {
    if (endpoint.u->node_id == destination)
      return endpoint.delay;
  }
  return NIL;
}

inline int GetEdgeResidualBandwidth(int source, int destination) {
  for (auto &endpoint : graph[source]) {
    if (endpoint.u->node_id == destination)
      return endpoint.residual_bandwidth;
  }
  return NIL;
}

inline int GetPathResidualBandwidth(int source, int destination) {
  std::vector<int> *path_ptr = nullptr;
  std::pair<int, int> cache_index(source, destination);
  if (path_cache[cache_index]) {
    path_ptr = path_cache[cache_index].get();
  } else {
    auto path = ComputeShortestPath(source, destination);
    path_cache[cache_index] = std::move(path);
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
  for (auto& mboxes : deployed_mboxes)
    mboxes.clear();
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

void UpdateMiddleboxInstances(int current_node, const middlebox& m_box, const
traffic_request& t_request) {
  bool new_provisioned = true;
  for (middlebox_instance& mbox_inst : deployed_mboxes[current_node]) {
    if (mbox_inst.m_box->middlebox_name == m_box.middlebox_name) {
      if (mbox_inst.residual_capacity >= t_request.min_bandwidth) {
        new_provisioned = false;
        mbox_inst.residual_capacity -= t_request.min_bandwidth;
      }
    }
  }
  if (new_provisioned) {
    deployed_mboxes[current_node].emplace_back(&m_box,
    m_box.processing_capacity);
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
    for (middlebox_instance& mbox_inst : deployed_mboxes[current_node]) {
      if (mbox_inst.m_box->middlebox_name == m_box.middlebox_name) {
        if (mbox_inst.residual_capacity >= t_request.min_bandwidth) {
          return 1;
        }
      }
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

inline double GetTransitCost(int prev_node, int current_node) {
  return 1.0 * shortest_path[prev_node][current_node] * per_bit_transit_cost;
}

inline double GetEnergyCost(const middlebox &m_box) {
  return per_core_cost * m_box.cpu_requirement;
}

inline double GetDeploymentCost(int current_node, const middlebox &m_box, const
traffic_request& t_request) {
  // If we can use existing middlebox then there is no deployment cost.
  for (middlebox_instance& mbox_inst : deployed_mboxes[current_node]) {
    if (mbox_inst.m_box->middlebox_name == m_box.middlebox_name) {
      if(mbox_inst.residual_capacity >= t_request.min_bandwidth) {
        return 0.0;
      }
    }
  }

  return m_box.deployment_cost;
}

double GetCost(int prev_node, int current_node, const middlebox &m_box,
               const traffic_request &t_request) {
  double deployment_cost = GetDeploymentCost(current_node, m_box, t_request);
  double energy_cost = GetEnergyCost(m_box);
  double transit_cost = GetTransitCost(prev_node, current_node);
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

std::unique_ptr<std::vector<int> >
ViterbiCompute(const traffic_request &t_request) {
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
      cost[stage][node] = GetCost(t_request.source, node, m_box, t_request);
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
              GetCost(prev_node, current_node, m_box, t_request);
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
        for (middlebox_instance& mbox_instance : 
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
        GetTransitCost(cur_node, t_request.destination) +
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
  return std::move(return_vector);
}

void UpdateResources(std::vector<int> *traffic_sequence,
                     const traffic_request &t_request) {
  for (int i = 0; i < static_cast<int>(traffic_sequence->size()) - 1; ++i) {
    ReducePathResidualBandwidth(traffic_sequence->at(i),
                                traffic_sequence->at(i + 1),
                                t_request.min_bandwidth);
  }
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

#endif //  MIDDLEBOX_PLACEMENT_SRC_VITERBI_H_
