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
  const std::string cost_ts_file_name = output_file_prefix + ".cost.ts";
  const std::string cost_summary_file_name = 
      output_file_prefix +  ".cost.summary";
  FILE* cost_ts_file = fopen(cost_ts_file_name.c_str(), "w");
  FILE* cost_summary_file = fopen(cost_summary_file_name.c_str(), "w");

  // Write the time series of cost data to file.
  int current_time = 0;
  std::vector<double> cost_data;
  std::vector<double> global_cost_data;
  s.t_stats.push_back(traffic_statistics(INF, INF));
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
  s.t_stats.pop_back();
  global_cost_data.pop_back();
  // Write the mean, 5th, and 95th percentile of the cost to file.
  double mean_cost = GetMean(global_cost_data);
  double fifth_percentile_cost = GetNthPercentile(global_cost_data, 5);
  double ninety_fifth_percentile_cost = GetNthPercentile(global_cost_data, 95);
  fprintf(cost_summary_file, "%.3lf %.3lf %.3lf\n", mean_cost,
          fifth_percentile_cost, ninety_fifth_percentile_cost);
  fclose(cost_ts_file);
  fclose(cost_summary_file);
  
}

#endif  // MIDDLEBOX_PLACEMENT_SRC_UTIL_H_
