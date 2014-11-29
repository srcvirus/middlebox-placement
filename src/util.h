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
    if (endpoint.u->node_id == destination) return endpoint.delay;
  }
  return NIL;
}

double GetSolutionStretch(const std::unique_ptr<std::vector<int>>& result) {
  int embedded_path_length = 0;
  const int kSequenceLength = result->size();
  int kSource = result->at(0);
  int kDestination = result->at(kSequenceLength - 1);
  int shortest_path_length = ComputeShortestPath(kSource, kDestination)->size()
                              - 1;
  for (int i = 0; i < kSequenceLength - 1; ++i) {
    embedded_path_length += ComputeShortestPath(result->at(i), 
                                                result->at(i + 1))->size() - 1;
  }
  return static_cast<double>(embedded_path_length) /
           static_cast<double>(shortest_path_length);
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
  std::vector<std::vector<double> > per_server_util;
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
