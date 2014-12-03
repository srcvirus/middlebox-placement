#!/bin/bash

folder_name=$1

cost_ts_filenames=""
util_ts_filenames=""
sequences_filenames=""
paths_filenames=""

for ti in {0..9775..630}
do
  sub_folder_name=$folder_name/run-$ti
  cost_ts_filenames="$cost_ts_filenames $sub_folder_name/log.cplex.cost.ts"
  util_ts_filenames="$util_ts_filenames $sub_folder_name/log.cplex.util.ts"
  sequences_filenames="$sequences_filenames $sub_folder_name/log.cplex.sequences"
  paths_filenames="$paths_filenames $sub_folder_name/log.cplex.paths"
done

cat $cost_ts_filenames > log.cplex.cost.ts
cat $util_ts_filenames > log.cplex.util.ts
cat $sequences_filenames > log.cplex.sequences
cat $paths_filenames > log.cplex.paths
