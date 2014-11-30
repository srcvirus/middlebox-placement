#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#define ARR_SIZE(arr) ((sizeof(arr)) / (sizeof(arr[0])))

using std::vector;
const static std::string middlebox_names[] = { "firewall", "proxy", "ids",
                                               "nat" };

vector<int> RandomSeqWithUniqueElements(int n) {
  vector<int> ret;
  int oldn = n;
  while (n) {
    bool found = false;
    int current = rand() % oldn;
    for (int val : ret) {
      if (val == current) {
        found = true;
        break;
      }
    }
    if (!found) {
      ret.push_back(current);
      --n;
    }
  }
  return ret;
}

void ReadLine(vector<double> &line, FILE **fpp) {
  const int kElementCount = 144;
  line.clear();
  for (int i = 0; i < kElementCount; ++i) {
    double element;
    fscanf(*fpp, "%lf", &element);
    line.push_back(element);
  }
}

vector<double> VectorDifference(const vector<double> &a,
                                const vector<double> &b) {
  vector<double> ret;
  for (int i = 0; i < a.size(); ++i)
    ret.push_back(a[i] - b[i]);
  return ret;
}

vector<double> ProcessWindow(const vector<vector<double>> v) {
  if (v.size() <= 1) return vector<double>();
  vector<double> ret(v[0].size(), 0);
  for (int i = 1; i < v.size(); ++i) {
    vector<double> diff = VectorDifference(v[i - 1], v[i]);
    for (int j = 0; j < diff.size(); ++j) {
      if (diff[j] <= 0) continue;
      ret[j] += diff[j];
    }
  }
  return ret;
}

int main(int argc, char *argv[]) {
  const int kMaxTrafficFileIndex = atoi(argv[1]);
  int current_time = 0;
  FILE *ofp = fopen("traffic-requests", "w");
  for (int i = 1, current_time = 0; i <= kMaxTrafficFileIndex; ++i) {
    std::string kTrafficFileName =
        "/home/sr2chowd/UW/middlebox-placement/traffic-data-yzhang/X";
    kTrafficFileName += (i <= 9 ? "0" : "") + std::to_string(i);
    printf("Openning %s\n", kTrafficFileName.c_str());
    FILE *ifp = fopen(kTrafficFileName.c_str(), "r");
    if (!ifp) {
      perror("Failed to open file\n");
      exit(0);
    }
    const int kLineCount = 7 * 24 * 12;
    const int kTimeWindow = 100;
    vector<double> current_line, prev_line;
    // ReadLine(prev_line, &ifp);
    int j = 0;
    while (j < kLineCount) {
      int n_points = 0;
      int start_time = current_time;
      int stop_time = current_time + kTimeWindow;
      vector<vector<double>> current_traffic_window;
      while (j < kLineCount &&
              current_time <= stop_time) {
        ReadLine(current_line, &ifp);
        current_traffic_window.push_back(current_line);
        current_time += 5;
        ++j;
        ++n_points;
      }
      vector<double> current_traffic = ProcessWindow(current_traffic_window);
      printf("current_traffc.size() = %lu\n", current_traffic.size());
      for (int k = 0; k < current_traffic.size(); ++k) {
        if (current_traffic[k] < 0)
          current_traffic[k] = 0;
        int u = k / 12;
        int v = k % 12;
        if (u == v)
          continue;
        current_traffic[k] /= (10.0 * 300.0 * (n_points - 1));
        int max_latency = 325 + 70 + (rand() % 30);
        double penalty = 0.01;
        int traffic = current_traffic[k];
        if (traffic < 300)
          continue;
        vector<int> mbox_seq = RandomSeqWithUniqueElements(3);
        if (traffic)
          // printf("%d,%d,%d,%d,%d,%0.3lf,%s,%s,%s\n", start_time, u, v,
          //      traffic, max_latency, penalty,
          //       middlebox_names[mbox_seq[0]].c_str(),
          //       middlebox_names[mbox_seq[1]].c_str(),
          //       middlebox_names[mbox_seq[2]].c_str());
        fprintf(ofp, "%d,%d,%d,%d,%d,%.3lf,%s,%s,%s\n", start_time, u, v,
                traffic, max_latency, penalty,
                middlebox_names[mbox_seq[0]].c_str(),
                middlebox_names[mbox_seq[1]].c_str(),
                middlebox_names[mbox_seq[2]].c_str());
      }
      printf("Traffic duration = %d seconds\n", 300 * (n_points - 1));
    }
    fclose(ifp);
  }
  fclose(ofp);
  return 0;
}
