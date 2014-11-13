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
  return element / static_cast<T>(kNumElements);
}

template <class T>
T GetNthPercentile(const std::vector<T>& data, int n) {
  sort(data.begin(), data.end());
  const size_t kNumElements = data.size();
  int rank = n * kNumElements;
  if (rank % 100) {
    rank = (rank / 100) + 1;
  } else rank /= 100;
  return data[rank];
}

#endif  // MIDDLEBOX_PLACEMENT_SRC_UTIL_H_
