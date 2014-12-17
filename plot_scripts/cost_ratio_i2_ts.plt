set terminal pdfcairo enhanced color font "Helvetica,8" dashed linewidth 4 rounded fontscale 1.0 size 3in,4in
set style fill solid border 0

set output "cost_ratio_i2_ts.pdf"

# set   autoscale                        # scale axes automatically
set xtics nomirror
set ytic nomirror
set tmargin 0
set lmargin 6 
set multiplot layout 3,1
set origin 0,0

set key top right 
set xr[0:10]
set xtics 2

set xlabel font "Helvetica,8"
set ylabel font "Helvetica,8"
set xtic font "Helvetica,8"
set ytic font "Helvetica,8"

set xlabel "Time (x10^3 min)"
set ytics 0.1 rotate by 90 offset 0,0.5
set yr[0.95:1.19]
set size 1.0, 0.39
plot    	"log.total_cost_ratio" using ($1/1000):2 title "Total Cost" w line lc rgb '#FA160E' lt 5

set ytics 1 rotate by 90 offset 0,0.5
set origin 0.00, 0.35
set size 1.00,0.3
# set key top left 
#unset label
#unset xtics
set title ""
set xlabel ""
# set ylabel "Cost Ratio"
unset xtics

set yr[0.4:1.20]
set ytics 0.5 rotate by 90 offset 0,0.5

plot    	"log.energy_cost_ratio" using ($1/1000):2 title "Energy Cost" w line lt 3


set origin 0.0, 0.61
set size 1.00,0.3

#unset label
#unset xtics
set title "Heuristic vs. CPLEX" 
set xlabel ""
set ylabel ""
unset xtics
set yr[0.95:1.45]
# set ytics 50

set ytics 0.2 rotate by 90 offset 0,0.5
plot    	"log.transit_cost_ratio" using ($1/1000):2 title "Transit Cost" w line lc rgb '#9FBB55' lt 1

# set origin 0.0, 0.655
# set size 1.0,0.24

#unset label
#unset xtics

#set title "Time(s) vs Cost Ratio (Heuristic / CPLEX)" 
#set xlabel ""
#set ylabel ""
#unset xtics
#set ytics 0.0002 rotate by  90
# set key top right
#set xzeroaxis lt 0 lw 0
#set xr[0:10000]
#set yr[0:0.0002]
# set ytics 50

#plot    	"log.viterbi.i2.cost.ts" using 1:6 title "SLA Violation Cost" w line lc rgb '#6F9BDD' lt 7


unset multiplot

