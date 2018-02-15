#ifndef MIDDLEBOX_PLACEMENT_SRC_VITERBI_H_
#define MIDDLEBOX_PLACEMENT_SRC_VITERBI_H_

#include "datastructure.h"
#include "util.h"
#include <algorithm>

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
      current_vector[i].sw_cap.push_back(nodes[j].residual_sw_cap);
    }
  }
  for (node = 0; node < kNumNodes; ++node) {
    const middlebox &m_box = middleboxes[t_request.middlebox_sequence[0]];
    if (IsResourceAvailable(t_request.source, node, current_vector[node], m_box,
                            t_request)) {
      cost[stage][node] = GetCost(t_request.source, node, current_vector[node],
                                  m_box, t_request);
      current_vector[node].cpu_cores[node] -= m_box.cpu_requirement;
      current_vector[node].sw_cap[node] -= t_request.min_bandwidth;
      DEBUG("[First stage] cost[stage = 0][node = %d] = %lf\n", node, cost[stage][node]);
    }
  }
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
              cost[stage - 1][prev_node] + GetCost(prev_node, current_node,
                                                   previous_vector[prev_node],
                                                   m_box, t_request);
          DEBUG(
              "[stage = %d, middlebox = %s, prev_node = %d, tr_cost = "
              "%lf]\n",
              stage, m_box.middlebox_name.c_str(), prev_node, transition_cost);
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
        current_vector[current_node].sw_cap =
            previous_vector[min_index].sw_cap;
        bool new_middlebox_deployed = true;

        // For comparison with Khaleesi we are not reusing mboxes.
        // for (middlebox_instance &mbox_instance :
        //      deployed_mboxes[current_node]) {
        //   if (mbox_instance.m_box->middlebox_name == m_box.middlebox_name &&
        //       mbox_instance.residual_capacity >= t_request.min_bandwidth) {
        //     new_middlebox_deployed = false;
        //     break;
        //   }
        // }

        if (new_middlebox_deployed) {
          current_vector[current_node].cpu_cores[current_node] -=
              m_box.cpu_requirement;
          if (current_node == min_index) {
            current_vector[current_node].sw_cap[current_node] -=
              t_request.min_bandwidth;
          }
        }
      } else {
        current_vector[current_node].cpu_cores.clear();
        current_vector[current_node].sw_cap.clear();
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
        GetSLAViolationCost(cur_node, t_request.destination, t_request,
                            fake_mbox);
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

#endif  //  MIDDLEBOX_PLACEMENT_SRC_VITERBI_H_
