#ifndef MIDDLEBOX_PLACEMENT_SRC_UTIL_H_
#define MIDDLEBOX_PLACEMENT_SRC_UTIL_H_

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

void PrintDebugMessage(const char* location, const char* fmt_string, ...) {
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
double GetMean(const std::vector<T>& data) {
  T sum = T(0);
  const size_t kNumElements = data.size();
  for (auto& element : data) sum += element;
  return sum / static_cast<T>(kNumElements);
}

template <class T>
T GetNthPercentile(const std::vector<T>& data, int n) {
  std::vector<T> temp_data_buffer = data;
  sort(temp_data_buffer.begin(), temp_data_buffer.end());
  const size_t kNumElements = data.size();
  int rank = n * kNumElements;
  if (rank % 100) {
    rank = (rank / 100) + 1;
  } else rank /= 100;
  --rank;
  return temp_data_buffer[rank];
}

void ProcessStats(solution_statistics& s, 
                  const std::string& output_file_prefix) {
  const std::string kCostTsFileName = output_file_prefix + ".cost.ts";
  const std::string kCostSummaryFileName = 
      output_file_prefix +  ".cost.summary";
  const std::string kUtilTsFileName = output_file_prefix + ".util.ts";

  // Process cost data.
  FILE* cost_ts_file = fopen(kCostTsFileName.c_str(), "w");
  int current_time = 0;
  std::vector<double> cost_data;
  std::vector<double> global_cost_data;
  s.t_stats.push_back(traffic_statistics(INF, INF));
  // First write the time series data to file. For each time instance write the
  // mean, 5th, and 95th percentile of cost.
  for (auto& t_stats : s.t_stats) {
    if (current_time != t_stats.arrival_time) {
      double mean_cost = GetMean(cost_data);
      double fifth_percentile_cost = GetNthPercentile(cost_data, 5);
      double ninety_fifth_percentile_cost = GetNthPercentile(cost_data, 95);
      fprintf(cost_ts_file, "%d %.3lf %.3lf %.3lf\n", current_time, mean_cost,
              fifth_percentile_cost, ninety_fifth_percentile_cost);
      current_time = t_stats.arrival_time;
      cost_data.clear();
    }
    global_cost_data.push_back(t_stats.cost);
    cost_data.push_back(t_stats.cost);
  }
  fclose(cost_ts_file);
  s.t_stats.pop_back();
  global_cost_data.pop_back();

  // Write the mean, 5th, and 95th percentile of the cost to file.
  double mean_cost = GetMean(global_cost_data);
  double fifth_percentile_cost = GetNthPercentile(global_cost_data, 5);
  double ninety_fifth_percentile_cost = GetNthPercentile(global_cost_data, 95);
  FILE* cost_summary_file = fopen(kCostSummaryFileName.c_str(), "w");
  fprintf(cost_summary_file, "%.3lf %.3lf %.3lf\n", mean_cost,
          fifth_percentile_cost, ninety_fifth_percentile_cost);
  fclose(cost_summary_file);

  // Process utilization data.
  FILE* util_ts_file = fopen(kUtilTsFileName.c_str(), "w");
  current_time = 0;
  std::vector<double> util_data;
  s.server_stats.emplace_back(INF, NIL, INF);
  DEBUG("bingo\n");
  for (auto& server_stat : s.server_stats) {
    if (current_time != server_stat.timestamp) {
      double mean_util = GetMean(util_data);
      double fifth_percentile_util = GetNthPercentile(util_data, 5);
      double ninety_fifth_percentile_util = GetNthPercentile(util_data, 95);
      fprintf(util_ts_file, "%d %.3lf %.3lf %.3lf\n", current_time, mean_util,
                            fifth_percentile_util, ninety_fifth_percentile_util);
      DEBUG("%d %.3lf %.3lf %.3lf\n", current_time, mean_util,
                            fifth_percentile_util, ninety_fifth_percentile_util);
      current_time = server_stat.timestamp;
      util_data.clear();
    }
    util_data.push_back(server_stat.utilization);
  }
  fclose(util_ts_file);
}
#endif  // MIDDLEBOX_PLACEMENT_SRC_UTIL_H_
