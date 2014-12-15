#!/bin/bash
rm log.dc*cost_ratio
awk 'FNR==NR{a[$1]=$2;next}{ print $1 FS $2/a[$1]}' log.cplex.dc.cost.ts log.viterbi.dc.cost.ts &>> log.dc.total_cost_ratio
awk 'FNR==NR{a[$1]=$4;next}{ print $1 FS $4/a[$1]}' log.cplex.dc.cost.ts log.viterbi.dc.cost.ts &>> log.dc.energy_cost_ratio
awk 'FNR==NR{a[$1]=$5;next}{ print $1 FS $5/a[$1]}' log.cplex.dc.cost.ts log.viterbi.dc.cost.ts &>> log.dc.transit_cost_ratio
# awk 'FNR==NR{a[$1]=$6;next}{ print $1 FS $6/a[$1]}' log.cplex.dc.cost.ts log.viterbi.dc.cost.ts &>> log.sla_cost_ratio
