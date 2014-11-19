#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#define ARR_SIZE(arr) ((sizeof(arr)) / (sizeof(arr[0])))

using std::vector;
/*const static traffic_class t_classes[] = {
  {
    .class_name = "gold",
    .min_bandwidth = 500,
    .max_delay = 150,
    .delay_penalty = 0.01
  },
  {
    .class_name = "silver",
    .min_bandwidth = 250,
    .max_delay = 200,
    .delay_penalty = 0.004
  },
  {
    .class_name = "bronze",
    .min_bandwidth = 200,
    .max_delay = 220,
    .delay_penalty = .001
  }
};*/
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
    // const int kLineCount = 5;
    vector<double> current_line, prev_line;
    ReadLine(prev_line, &ifp);
    for (int j = 1; j < kLineCount; ++j, current_time += 5) {
      ReadLine(current_line, &ifp);
      vector<double> current_traffic =
          VectorDifference(current_line, prev_line);
      printf("current_traffc.size() = %lu\n", current_traffic.size());
      for (int k = 0; k < current_traffic.size(); ++k) {
        if (current_traffic[k] < 0)
          current_traffic[k] = 0;
        int u = k / 12;
        int v = k % 12;
        if (u == v)
          continue;
        current_traffic[k] /= (10.0 * 300);
        int max_latency = 290 + (rand() % 20);
        double penalty = 0.01;
        int traffic = current_traffic[k];
        if (traffic < 300)
          continue;
        vector<int> mbox_seq = RandomSeqWithUniqueElements(3);
        if (traffic)
          printf("%d,%d,%d,%d,%d,%0.3lf,%s,%s,%s\n", current_time, u, v, traffic,
                 max_latency, penalty, middlebox_names[mbox_seq[0]].c_str(),
                 middlebox_names[mbox_seq[1]].c_str(),
                 middlebox_names[mbox_seq[2]].c_str());
        fprintf(ofp, "%d,%d,%d,%d,%d,%.3lf,%s,%s,%s\n", current_time, u, v, traffic,
                max_latency, penalty, middlebox_names[mbox_seq[0]].c_str(),
                middlebox_names[mbox_seq[1]].c_str(),
                middlebox_names[mbox_seq[2]].c_str());
      }
      prev_line = current_line;
    }
    fclose(ifp);
  }
  fclose(ofp);
  return 0;
}
