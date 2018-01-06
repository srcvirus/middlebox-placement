#ifndef MIDDLEBOX_PLACEMENT_SRC_IO_H_
#define MIDDLEBOX_PLACEMENT_SRC_IO_H_

#include "datastructure.h"
#include "util.h"
#include <algorithm>
#include <string.h>

std::unique_ptr<std::map<std::string, std::string> > ParseArgs(int argc,
                                                               char *argv[]) {
  std::unique_ptr<std::map<std::string, std::string> > arg_map(
      new std::map<std::string, std::string>());
  for (int i = 1; i < argc; ++i) {
    char *key = strtok(argv[i], "=");
    char *value = strtok(NULL, "=");
    DEBUG(" [%s] => [%s]\n", key, value);
    arg_map->insert(std::make_pair(key, value));
  }
  return std::move(arg_map);
}

inline int GetMiddleboxIndex(const std::string &middlebox_name) {
  DEBUG("Finding middlebox: %s\n", middlebox_name.c_str());
  for (int i = 0; i < middleboxes.size(); ++i) {
    DEBUG("i = %d, name = %s\n", i, middleboxes[i].middlebox_name.c_str());
    if (middleboxes[i].middlebox_name == middlebox_name) {
      DEBUG("Middlebox %s found at %d\n", middlebox_name.c_str(), i);
      return i;
    }
  }
  DEBUG("Middlebox %s not found :(\n", middlebox_name.c_str());
  return -1;  // Not found.
}

std::unique_ptr<std::vector<std::vector<std::string> > > ReadCSVFile(
    const char *filename) {
  DEBUG("[Parsing %s]\n", filename);
  FILE *file_ptr = fopen(filename, "r");
  const static int kBufferSize = 1024;
  char line_buffer[kBufferSize];
  std::unique_ptr<std::vector<std::vector<std::string> > > ret_vector(
      new std::vector<std::vector<std::string> >());
  std::vector<std::string> current_line;
  int row_number = 0;
  while (fgets(line_buffer, kBufferSize, file_ptr)) {
    current_line.clear();
    char *token = strtok(line_buffer, ",\n\r");
    current_line.push_back(token);
    while ((token = strtok(NULL, ",\n"))) {
      current_line.push_back(token);
    }
    ret_vector->push_back(current_line);
  }
  fclose(file_ptr);
  DEBUG("Parsed %d lines\n", static_cast<int>(ret_vector->size()));
  return std::move(ret_vector);
}

void InitializeAllResults(const char *filename) {
  results.clear();
  auto csv_vector = ReadCSVFile(filename);
  for (int i = 0; i < csv_vector->size(); ++i) {
    std::vector<std::string> &row = (*csv_vector)[i];
    std::vector<int> current_result;
    for (auto &element : row) {
      current_result.push_back(atoi(element.c_str()));
      DEBUG("Pushing %s\n", element.c_str());
    }
    DEBUG("\n");
    results.push_back(current_result);
  }
}

void InitializeSolutionPaths(const char *filename) {
  paths.clear();
  auto csv_vector = ReadCSVFile(filename);
  for (int i = 0; i < csv_vector->size(); ++i) {
    std::vector<std::string> &row = (*csv_vector)[i];
    std::vector<int> current_path;
    for (auto &element : row) {
      current_path.push_back(atoi(element.c_str()));
    }
    paths.push_back(current_path);
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

void PrintTrafficRequests() {
  printf("[Traffic Requests (count = %d)\n",
         static_cast<int>(traffic_requests.size()));
  for (int i = 0; i < traffic_requests.size(); ++i) {
    printf("[i = %d] %s\n", i, traffic_requests[i].GetDebugString().c_str());
  }
}

void InitializeTrafficRequests(const char *filename) {
  traffic_requests.clear();
  auto csv_vector = ReadCSVFile(filename);
  for (int i = 0; i < csv_vector->size(); ++i) {
    std::vector<int> mbox_sequence;
    std::vector<std::string> &row = (*csv_vector)[i];
    for (int mbox_seq_index = 6; mbox_seq_index < row.size();
         ++mbox_seq_index) {
      mbox_sequence.push_back(GetMiddleboxIndex(row[mbox_seq_index]));
    }
    traffic_requests.emplace_back(row[0], row[1], row[2], row[3], row[4],
                                  row[5], mbox_sequence);
  }
  int last_time_stamp = max_time;
  int current_time = traffic_requests.back().arrival_time;
  for (int i = traffic_requests.size() - 1; i >= 0; --i) {
    if (current_time != traffic_requests[i].arrival_time) {
      last_time_stamp = current_time;
      current_time = traffic_requests[i].arrival_time;
    }
    traffic_requests[i].duration =
        (last_time_stamp - traffic_requests[i].arrival_time) * 60;
  }
}

void InitializeTopology(const char *filename) {
  DEBUG("[Parsing %s]\n", filename);
  FILE *file_ptr = fopen(filename, "r");
  int node_count, edge_count;
  int ret_val = fscanf(file_ptr, "%d %d", &node_count, &edge_count);
  DEBUG(" node_count = %d, edge_count = %d\n", node_count, edge_count);
  graph.resize(node_count);
  nodes.resize(node_count);
  deployed_mboxes.resize(node_count);
  for (int i = 0; i < node_count; ++i) {
    ret_val = fscanf(file_ptr, "%d %d", &nodes[i].node_id, &nodes[i].num_cores);
    nodes[i].residual_cores = nodes[i].num_cores;
    graph[i].clear();
    shortest_path[i][i] = 0.0;
    shortest_edge_path[i][i] = 0;
    bw[i][i] = 0;
    sp_pre[i][i] = NIL;
    for (int j = i + 1; j < node_count; j++) {
      shortest_path[i][j] = shortest_path[j][i] = INF;
      shortest_edge_path[i][j] = shortest_edge_path[j][i] = INF;
      sp_pre[i][j] = sp_pre[j][i] = NIL;
      bw[i][j] = bw[j][i] = 0;
    }
  }
  DEBUG("nodes.size() = %u\n", nodes.size());
  for (auto &n : nodes) {
    DEBUG("%s\n", n.GetDebugString().c_str());
  }

  for (int j = 0; j < edge_count; ++j) {
    int source, destination, delay;
    unsigned long bandwidth;
    ret_val = fscanf(file_ptr, "%d %d %lu %d", &source, &destination, &bandwidth, &delay);
    DEBUG(" Read edge: %d %d %lu %d\n", source, destination, bandwidth, delay);
    DEBUG(" Adding edge, %d --> %s\n", source,
          nodes[destination].GetDebugString().c_str());
    DEBUG(" Adding edge, %d --> %s\n", destination,
          nodes[source].GetDebugString().c_str());
    graph[source].emplace_back(&nodes[destination], bandwidth, delay);
    graph[destination].emplace_back(&nodes[source], bandwidth, delay);
    bw[source][destination] = bw[destination][source] = bandwidth;
    shortest_edge_path[source][destination] = 1;
    shortest_edge_path[destination][source] = 1;
    shortest_path[source][destination] = shortest_path[destination][source] =
        delay;
    sp_pre[source][destination] = source;
    sp_pre[destination][source] = destination;
  }
  for (int k = 0; k < node_count; ++k) {
    for (int i = 0; i < node_count; ++i) {
      for (int j = 0; j < node_count; ++j) {
        if (i == j) continue;
        int relaxed_cost = shortest_edge_path[i][k] + shortest_edge_path[k][j];
        if (shortest_edge_path[i][j] > relaxed_cost) {
          shortest_edge_path[i][j] = relaxed_cost;
          shortest_path[i][j] = shortest_path[i][k] + shortest_path[k][j];
          sp_pre[i][j] = sp_pre[k][j];
        }
      }
    }
  }
  closeness.resize(node_count);
  for (int i = 0; i < node_count; ++i) {
    double farness = 0.0;
    for (int j = 0; j < node_count; ++j) {
      if (i != j) {
        farness += shortest_edge_path[i][j];
      }
    }
    closeness[i] = 1.0 / farness;
  }
  fclose(file_ptr);
}

#endif  // MIDDLEBOX_PLACEMENT_SRC_IO_H_
