#!/bin/bash

# create directory with timestamp
folder_name=$(date +%Y%m%d_%H%M%S)
mkdir -p /home/mfbari/middlebox-placement/src/$folder_name

# start time instance 
start_time_instance=0
end_time_instance=9975
time_instance_interval=105
time_instance_per_file=6

#loop
for ti in {0..9775..630}
do
  nti=$((ti+time_instance_interval*time_instance_per_file))
  start_line=$((`grep -n -m 1 "^$ti" traffic-request | awk -F: '{print $1}'`))
  end_line=$((`grep -n -m 1 "^$nti" traffic-request | awk -F: '{print $1}'`-1))

  if [[ $end_line -eq "-1" ]]
  then
    end_line=`wc -l < traffic-request`
  fi

  #echo $ti " " $nti
  #echo $start_line " " $end_line

  sub_folder_name=$folder_name/run-$ti
  `mkdir -p /home/mfbari/middlebox-placement/src/$sub_folder_name`

  # copy all input files in the subfolder
  `cp middleman cplex_run topology middlebox-spec "$sub_folder_name"/`
  `sed -n "$start_line","$end_line"p traffic-request > "$sub_folder_name"/traffic-request`

  # run the processes
  (cd /home/mfbari/middlebox-placement/src/"$sub_folder_name" && `cat cplex_run` > output &)
done
