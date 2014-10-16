#include "datastructure.h"
#include <map>
#include <utility>
#include <memory>
#include <stdio.h>
#include <string>
#include <string.h>

const std::string kUsage = "./middleman --per_core_cost=<per_core_cost>\n\t--per_bit_transit_cost=<per_bit_transit_cost>\n\t--topology_file=<topology_file>\n\t--traffic_class_file=<traffic_class_file>\n\t--middlebox_spec_file=<middlebox_spec_file>\n\t--traffic_request_file=<traffic_request_file>";

std::vector<middlebox> middleboxes;
std::vector<traffic_class> traffic_classes;
std::vector<traffic_request> traffic_requests;
std::vector<node> nodes;
std::vector<std::vector<edge_endpoint> > graph;
double per_core_cost, per_bit_transit_cost;

std::unique_ptr<std::map<std::string,std::string>> ParseArgs(
  int argc, char* argv[]) {
  std::unique_ptr<std::map<std::string,std::string>> arg_map(
      new std::map<std::string,std::string>());
  for (int i = 1; i < argc; ++i) {
    char* key = strtok(argv[i], "=");
    char* value = strtok(NULL, "=");
#ifdef DEBUG
    printf(" [%s] => [%s]\n", key, value);
#endif
    arg_map->insert(std::make_pair(key, value));
  }
  return std::move(arg_map);
}

inline int GetMiddleboxIndex(const std::string &middlebox_name) {
  for (int i = 0; i < middleboxes.size(); ++i) {
    if (middleboxes[i].middlebox_name == middlebox_name)
      return i;
  }
  return -1; // Not found.
}

inline int GetTrafficClassIndex(const std::string &traffic_class_name) {
  for (int i = 0; i < traffic_classes.size(); ++i) {
    if (traffic_classes[i].class_name == traffic_class_name)
      return i;
  }
  return -1; // Not found.
}

std::unique_ptr<std::vector<std::vector<std::string> > >
ReadCSVFile(const char *filename) {
#ifdef DEBUG
  printf("[Parsing %s]\n", filename);
#endif
  FILE *file_ptr = fopen(filename, "r");
  const static int kBufferSize = 1024;
  char line_buffer[kBufferSize];
  std::unique_ptr<std::vector<std::vector<std::string> > > ret_vector(
      new std::vector<std::vector<std::string> >());
  std::vector<std::string> current_line;
  int row_number = 0;
  while (fgets(line_buffer, kBufferSize, file_ptr)) {
    current_line.clear();
    char *token = strtok(line_buffer, ",");
    current_line.push_back(token);
    while ((token = strtok(NULL, ","))) {
      current_line.push_back(token);
    }
#ifdef DEBUG
    for (std::string t : current_line) printf(" %s", t.c_str());
#endif
    ret_vector->push_back(current_line);
  }
  fclose(file_ptr);
#ifdef DEBUG
  printf("Parsed %d lines\n", static_cast<int>(ret_vector->size()));
#endif
  return std::move(ret_vector);
}

void InitializeTrafficClasses(const char *filename) {
  traffic_classes.clear();
  auto csv_vector = ReadCSVFile(filename);
  for (int i = 0; i < csv_vector->size(); ++i) {
    std::vector<std::string> &row = (*csv_vector)[i];
    traffic_classes.emplace_back(row[0], row[1], row[2], row[3]);
  }
}

void PrintTrafficClasses() {
  printf("[Traffic Classes (count = %d)]\n",
          static_cast<int>(traffic_classes.size()));
  for (int i = 0; i < traffic_classes.size(); ++i) {
    printf("[i = %d] %s\n", i, traffic_classes[i].GetDebugString().c_str());
  }
}

void InitializeMiddleboxes(const char *filename) {
  middleboxes.clear();
  auto csv_vector = ReadCSVFile(filename);
  for (int i = 0; i < csv_vector->size(); ++i) {
    std::vector<std::string> &row = (*csv_vector)[i];
    middleboxes.emplace_back(row[0], row[1], row[2], row[3], row[4]);
  }
}

void PrintMiddleboxes() {
  printf("[Middleboxes (count = %d)]\n", static_cast<int>(middleboxes.size()));
  for (int i = 0; i < middleboxes.size(); ++i) {
    printf("[i = %d] %s\n", i, middleboxes[i].GetDebugString().c_str());
  }
}

void InitializeTrafficRequests(const char *filename) {
  traffic_requests.clear();
  auto csv_vector = ReadCSVFile(filename);
  for (int i = 0; i < csv_vector->size(); ++i) {
    std::vector<int> mbox_sequence;
    std::vector<std::string> &row = (*csv_vector)[i];
    // row[2] contains the name of the traffic class.
    int sla_specification = GetTrafficClassIndex(row[2]);

    for (int mbox_seq_index = 3; mbox_seq_index < row.size();
         ++mbox_seq_index) {
      mbox_sequence.push_back(GetMiddleboxIndex(row[mbox_seq_index]));
    }
    traffic_requests.emplace_back(row[0], row[1], sla_specification,
                                 mbox_sequence);
  }
}

void PrintTrafficRequests() {
  printf("[Traffic Requests (count = %d)\n",
         static_cast<int>(traffic_requests.size()));
  for (int i = 0; i < traffic_requests.size(); ++i) {
    printf("[i = %d] %s\n", i, traffic_requests[i].GetDebugString().c_str());
  }
}

void InitializeTopology(const char* filename) {
#ifdef DEBUG
  printf("[Parsing %s]\n", filename);
#endif
  FILE* file_ptr = fopen(filename, "r");
  int node_count, edge_count;
  fscanf(file_ptr, "%d %d", &node_count, &edge_count);
#ifdef DEBUG
  printf(" node_count = %d, edge_count = %d\n", node_count, edge_count);
#endif
  graph.resize(node_count);
  nodes.resize(node_count);
  for (int i = 0; i < node_count; ++i) {
    fscanf(file_ptr,"%d %d", &nodes[i].node_id, &nodes[i].num_cores);
    nodes[i].residual_cores = node[i].num_cores;
    graph[i].clear();
    shortest_path[i][i] = 0.0;
    for (int j = i + 1; j < node_count; j++) {
      shortest_path[i][j] = shortest_path[j][i] = INF;
    }
  }

  for (int j = 0; j < edge_count; ++j) {
    int source, destination, bandwidth, delay;
    fscanf(file_ptr, "%d %d %d %d", &source, &destination, &bandwidth, &delay);
#ifdef DEBUG
    printf(" Adding edge, %d --> %s\n", source,
           nodes[destination].GetDebugString().c_str());
    printf(" Adding edge, %d --> %s\n", destination,
           nodes[source].GetDebugString().c_str());
#endif
    graph[source].emplace_back(&nodes[destination], bandwidth, delay);
    graph[destination].emplace_back(&nodes[source], bandwidth, delay);
    shortest_path[source][destination] = shortest_path[destination][source] =
      delay;
  }
  for (int k = 0; k < node_count; ++k) {
    for (int i = 0; i < node_count; ++i) {
      for (int j = 0; j < node_count; ++j) {
        shortest_path[i][j] = std::min(
                                shortest_path[i][j],
                                shortest_path[i][k] + shortest_path[k][j]);
      }
    }
  }
}

int IsResourceAvailable(const resource& resource_vector, int candidate,
                        const middlebox& m_box) {
  if (resource_vector.cpu_cores[candidate] >= m_box.cpu_requirement)
    return 1;
  return 0;
}

inline double GetSLAViolationCost(int prev_node, int current_node,
                           const traffic_request& t_request) {
  const int kNumSegments = t_request.middlebox_sequence.size() + 1;
  const double kPerSegmentLatencyBound = 
    (1.0 * traffic_classes[t_request.sla_specification].max_delay) /
    kNumSegments;
  if (shortest_path[prev_node][current_node] > kPerSegmentLatencyBound)
    return (shortest_path[prev_node][current_node] - kPerSegmentLatencyBound) *
             traffic_classes[t_request.sla_specification].delay_penalty;
  return 0.0;
}

inline double GetTransitCost(int prev_node, int current_node) {
  return 1.0 * shortest_path[prev_node][current_node] * per_bit_transit_cost;
}

inline double GetEnergyCost(const middlebox& m_box) {
  return per_core_cost * m_box.cpu_requirement;
}
                           
double GetCost(int prev_node, int current_node,
               const middlebox& m_box,
               const traffic_request& t_request) {
  double deployment_cost = m_box.deployment_cost;
  double energy_cost = GetEnergyCost(m_box);
  double transit_cost = GetTransitCost(prev_node, current_node);
  double sla_violation_cost = GetSLAViolationCost(prev_node, current_node,
                                                  t_request);
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

std::unique_ptr<std::vector<int> > ViterbiCompute(const traffic_request& t_request) {
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
    if (IsResourceAvailable(current_vector, node,
                            t_request.middlebox_sequence[0])) {
      cost[stage][node] = GetCost(t_request.source, node,
        t_request.middlebox_sequence[0], t_request);
      current_vector[node].cpu_cores[node] -=
        t_request.middlebox_sequence[0].cpu_requirement;
    }
  }
  // TODO(shihab): Handle repeated middleboxes.
  for (stage = 1; stage < kNumStages; ++stage) {
    previous_vector = current_vector;
    for (int current_node = 0; current_node < kNumNodes; ++current_node) {
      int min_index = NIL;
      for (int prev_node = 0; prev_node < kNumNodes; ++prev_node) {
        if (IsResourceAvailable(previous_vector[prev_node], current_node,
                                t_request.middlebox_sequence[stage])) {
          double transition_cost = cost[stage - 1][prev_node] +
                                   GetCost(prev_node, current_node,
                                           t_request.middlebox_sequence[stage],
                                           t_request);
          if (cost[stage][current_node] > transition_cost) {
            cost[stage][current_node] = transition_cost;
            pre[stage][current_node] = prev_node;
            min_index = prev_node;
          }
        }
      }
      current_vector[current_node].cpu_cores =
        previous_vector[min_index].cpu_cores;
      current_vector[current_node].cpu_cores[current_node] -=
        t_request.middlebox_sequence[stage].cpu_requirement;
    }
  }
  double min_cost = INF;
  double min_index = NIL;
  for (node = 0; node < kNumNodes; ++node) {
    double transition_cost = cost[kNumStages - 1][node] + 
                             GetTransportCost(node, t_request.destination) +
                             GetSLAViolationCost(node, t_request.destination,
                                                 t_request);
    if (min_cost > transit_cost) {
      min_cost = transit_cost;
      min_index = node;
    }
  }
  std::unique_ptr<vector<int> > return_vector(new vector<int>());
  return_vector->resize(kNumStages);
  int current_node = min_index;
  for (stage = kNumStages - 1; stage >= 0; --stage) {
    return_vector->push_back(current_node);
    current_node = pre[stage][current_node];
  }
  std::reverse(return_vector->begin(), return_vector->end());
  return std::move(return_vector);
}

void ViterbiPrintSolution() {
}

int main(int argc, char* argv[]) {
  if (argc < 7) {
    puts(kUsage.c_str());
    return 1;
  }
  auto arg_maps = ParseArgs(argc, argv);
  for (auto argument : *arg_maps) {
    if (argument.first == "--per_core_cost") {
      per_core_cost = atof(argument.second.c_str());
    }
    else if (argument.first == "--per_bit_transit_cost") {
      per_bit_transit_cost = atof(argument.second.c_str());
    }
    else if (argument.first == "--topology_file") {
      InitializeTopology(argument.second.c_str());
    }
    else if (argument.first == "--traffic_class_file") {
      InitializeTrafficClasses(argument.second.c_str());
      PrintTrafficClasses();
    }
    else if (argument.first == "--middlebox_spec_file") {
      InitializeMiddleboxes(argument.second.c_str());
      PrintMiddleboxes();
    }
    else if (argument.first == "--traffic_request_file") {
      InitializeTrafficRequests(argument.second.c_str());
      PrintTrafficRequests();
    }
  }
  return 0;
}












































