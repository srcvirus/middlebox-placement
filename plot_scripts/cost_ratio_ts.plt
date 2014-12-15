set terminal pdfcairo enhanced color font "Helvetica,9" dashed linewidth 4 rounded fontscale 1.0 size 4in,5in
set style data histogram
set style histogram cluster gap 1 
set style fill solid border 0

set output "cost_ratio_ts.pdf"

# set   autoscale                        # scale axes automatically
set xtics nomirror
set ytic nomirror
set tmargin 0
set lmargin 6 
set multiplot layout 4,1
set origin 0,0

set key right bottom
set xr[0:10000]
set yr[0.4:1.50]
set ytics 0.6 rotate by 90 offset 0,0.5

set xlabel font "Helvetica,8"
set ylabel font "Helvetica,8"
set xtic font "Helvetica,8"
set ytic font "Helvetica,8"

set size 1.0, 0.28
plot    	"log.total_cost_ratio" using 1:2 title "Total Cost Ratio" w line lc rgb '#FA160E' lt 5

set origin 0.00, 0.24
set size 1.00,0.245

#unset label
#unset xtics
set title ""
set xlabel ""
# set ylabel "Cost Ratio"
unset xtics
set xr[0:10000]
set yr[0.4:1.5]
# set ytics 50

plot    	"log.energy_cost_ratio" using 1:2 title "Energy Cost Ratio" w line lt 3


set origin 0.0, 0.45
set size 1.00,0.24

#unset label
#unset xtics
set title ""
set xlabel ""
set ylabel ""
unset xtics
set xr[0:10000]
set yr[0.4:1.5]
# set ytics 50

plot    	"log.transit_cost_ratio" using 1:2 title "Transit Cost Ratio" w line lc rgb '#9FBB55' lt 1

set origin 0.0, 0.655
set size 1.0,0.24

#unset label
#unset xtics

set title "Time(s) vs Cost Ratio (Heuristic / CPLEX)" 
set xlabel ""
set ylabel ""
unset xtics
set ytics 0.0001 rotate by  90
set key top right
#set xzeroaxis lt 0 lw 0
set xr[0:10000]
set yr[0:0.0001]
# set ytics 50

plot    	"log.viterbi.cost.ts" using 1:6 title "SLA Violation Cost" w line lc rgb '#6F9BDD' lt 7


unset multiplot

